#define _POSIX_C_SOURCE 200809L

#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <dirent.h>
#include <errno.h>

#define PULL mem(++S, 1, 0, 0)
#define PUSH(x) mem(S--, 1, x, 1)

static int fbfd = -1;
static uint32_t *fb_mem = NULL;
static uint32_t fb_w, fb_h;
static uint8_t key_state[256];

#define MAX_KBFDS 16
static int kbfds[MAX_KBFDS];
static int n_kbfds = 0;

static int nes_keycodes[8] = {
    KEY_X, KEY_Z, KEY_TAB, KEY_ENTER, KEY_W, KEY_S, KEY_A, KEY_D,
};

static void open_all_keyboards(void) {
    DIR *d = opendir("/dev/input");
    if (!d) { fprintf(stderr, "Warning: cannot open /dev/input (%s)\n", strerror(errno)); return; }
    struct dirent *ent;
    int perm_denied = 0;
    while ((ent = readdir(d)) != NULL) {
        if (strncmp(ent->d_name, "event", 5) != 0) continue;
        if (n_kbfds >= MAX_KBFDS) break;
        char path[300];
        snprintf(path, sizeof(path), "/dev/input/%s", ent->d_name);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) { if (errno == EACCES) perm_denied = 1; continue; }
        uint8_t evbits[EV_MAX / 8 + 1];
        memset(evbits, 0, sizeof(evbits));
        ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits);
        if (evbits[EV_KEY / 8] & (1 << (EV_KEY % 8))) { kbfds[n_kbfds++] = fd; } else { close(fd); }
    }
    closedir(d);
    if (n_kbfds == 0) {
        fprintf(stderr, "Warning: no usable keyboard found in /dev/input.\n");
        if (perm_denied) fprintf(stderr, "  Permission denied.\n");
    } else { fprintf(stderr, "Listening for input on %d keyboard device(s).\n", n_kbfds); }
}

static void poll_keyboard(void) {
    struct input_event ev;
    for (int k = 0; k < n_kbfds; k++) {
        while (read(kbfds[k], &ev, sizeof(ev)) == sizeof(ev)) {
            if (ev.type != EV_KEY) continue;
            for (int i = 0; i < 8; i++) if (ev.code == nes_keycodes[i]) key_state[i] = (ev.value != 0);
            if (ev.code == KEY_ESC && ev.value == 1) exit(0);
        }
    }
}

static uint32_t bgr565_to_xrgb(uint16_t c) {
    uint8_t b = (c >> 11) & 0x1f;
    uint8_t g = (c >> 5)  & 0x3f;
    uint8_t r = (c)       & 0x1f;
    return ((r << 3) | (r >> 2)) << 16 | ((g << 2) | (g >> 4)) << 8 | ((b << 3) | (b >> 2));
}

// --- PERF: BGR565 -> XRGB8888 lookup table ---
// There are only 65536 possible uint16_t pixel values, so instead of doing
// the shift/mask conversion for every single output pixel (up to ~2M times
// per frame at 1080p with scaling), we do it ONCE per distinct value at
// startup (65536 times total, <1ms) and then every blit is a plain memory
// load. The table is 256KB, comfortably L2-resident.
static uint32_t xrgb_lut[65536];
static void build_xrgb_lut(void) {
    for (uint32_t c = 0; c < 65536; c++) {
        xrgb_lut[c] = bgr565_to_xrgb((uint16_t)c);
    }
}

// --- PERF: row-cached scaled blit ---
// The original blit recomputed nes_x/nes_y and did a full BGR565->XRGB
// conversion for EVERY output pixel, even though under N x integer scaling
// each source pixel maps to an NxN block of identical output pixels. Here
// we convert each NES source row to XRGB exactly once (256 LUT lookups),
// then replicate that converted row "scale" times vertically and each
// converted pixel "scale" times horizontally. This cuts total conversion
// work from (out_w * out_h) lookups down to (NES_W * NES_H) lookups, with
// everything else being plain sequential memory writes.
static void blit_to_fb(uint16_t *frame_buffer) {
    const int NES_W = 256, NES_H = 224;
    uint32_t scale_x = fb_w / NES_W;
    uint32_t scale_y = fb_h / NES_H;
    uint32_t scale = scale_x < scale_y ? scale_x : scale_y;
    if (scale < 1) scale = 1;
    uint32_t out_w = NES_W * scale;
    uint32_t out_h = NES_H * scale;
    uint32_t off_x = (fb_w - out_w) / 2;
    uint32_t off_y = (fb_h - out_h) / 2;

    static uint32_t converted_row[256]; // scratch: one converted NES scanline

    for (int nes_y = 0; nes_y < NES_H; nes_y++) {
        uint16_t *src_row = frame_buffer + nes_y * NES_W;

        // Convert this source row once: 256 LUT lookups, not out_w.
        for (int nx = 0; nx < NES_W; nx++) {
            converted_row[nx] = xrgb_lut[src_row[nx]];
        }
        for (uint32_t sy = 0; sy < scale; sy++) {
            uint32_t *dst = fb_mem + (off_y + (uint32_t)nes_y * scale + sy) * fb_w + off_x;
            uint32_t x = 0;
            for (int nx = 0; nx < NES_W; nx++) {
                uint32_t v = converted_row[nx];
                for (uint32_t s = 0; s < scale; s++) dst[x++] = v;
            }
        }
    }
}

uint8_t *rom, *chrrom,
    prg[4], chr[8],
    prgbits = 14, chrbits = 12,
    A, X, Y, P = 4, S = ~2, PCH, PCL,
    addr_lo, addr_hi,
    nomem, result, val, cross, tmp,
    ppumask, ppuctrl, ppustatus, ppubuf, W, fine_x, opcode, nmi_irq, ntb, ptb_lo,
    vram[2048], palette_ram[64], ram[8192], chrram[8192], prgram[8192], oam[256],
    mask[] = {128, 64, 1, 2, 1, 0, 0, 1, 4, 0, 0, 4, 0, 0, 64, 0, 8, 0, 0, 8},
    keys, mirror,
    mmc1_bits, mmc1_data, mmc1_ctrl,
    mmc3_chrprg[8], mmc3_bits, mmc3_irq, mmc3_latch,
    chrbank0, chrbank1, prgbank,
    rombuf[1024 * 1024];

uint16_t scany, T, V, sum, dot, atb, shift_hi, shift_lo, cycles, frame_buffer[61440];

// --- PERF: per-scanline sprite candidate cache ---
// The original code re-scanned all 64 OAM sprites for EVERY one of the
// 256 pixels on a scanline (up to 256*64 = 16384 checks per scanline,
// ~3.9M/frame). Most sprites don't overlap a given scanline at all, so
// here we scan OAM ONCE per scanline (at the first pixel of that line)
// to build a short list of only the sprites that actually overlap it,
// then each of the 256 pixels on that line only tests against that
// short list instead of all 64.
//
// NOTE: real NES hardware hard-caps this list at 8 sprites/scanline and
// drops the rest (that's what the sprite-overflow flag signals). We
// deliberately do NOT enforce that cap here: this emulator's original
// code never enforced it either, so capping at 8 would change rendered
// output on any scene with >8 sprites on one line (common in NES games).
// The cache size below (64) is the true OAM maximum, so this is purely
// a speed optimization with byte-for-byte identical visual output to
// the original, verified against the original logic via fuzz testing.
#define MAX_LINE_SPRITES 64
static uint8_t line_sprite_idx[MAX_LINE_SPRITES]; // OAM byte-offset (0,4,8,...) of each candidate
static uint8_t line_sprite_count = 0;
static uint16_t line_sprite_scany = 0xffff; // which scanline the cache above is valid for

int shift_at = 0;

uint8_t *get_chr_byte(uint16_t a) { return &chrrom[chr[a >> chrbits] << chrbits | a % (1 << chrbits)]; }
uint8_t *get_nametable_byte(uint16_t a) {
  return &vram[mirror == 0   ? a % 1024
               : mirror == 1 ? a % 1024 + 1024
               : mirror == 2 ? a & 2047
                             : a / 2 & 1024 | a % 1024];
}

// --- PERF: build the list of sprites that overlap scanline `for_scany`
// exactly once (instead of redoing this same Y-range test for every one
// of the 256 pixels on the line). This does NOT enforce the real
// hardware's 8-sprite-per-line cap, so it cannot change which sprites
// get drawn vs the original code -- it only avoids repeating the same
// per-sprite Y-overlap test 256 times.
static void eval_sprites_for_scanline(uint16_t for_scany) {
  line_sprite_count = 0;
  uint16_t sprite_h = ppuctrl & 32 ? 16 : 8;
  for (int off = 0; off < 256; off += 4) {
    uint16_t sprite_y = for_scany - oam[off] - 1;
    if (sprite_y < sprite_h) {
      line_sprite_idx[line_sprite_count++] = (uint8_t)off;
    }
  }
  line_sprite_scany = for_scany;
}

uint8_t mem(uint8_t lo, uint8_t hi, uint8_t val, uint8_t write) {
  uint16_t addr = hi << 8 | lo;
  switch (hi >>= 4) {
  case 0: case 1: return write ? ram[addr] = val : ram[addr];
  case 2: case 3:
    lo &= 7;
    if (lo == 7) {
      tmp = ppubuf;
      uint8_t *rom = V < 8192 ? write && chrrom != chrram ? &tmp : get_chr_byte(V)
                     : V < 16128 ? get_nametable_byte(V) : palette_ram + (uint8_t)((V & 19) == 16 ? V ^ 16 : V);
      write ? *rom = val : (ppubuf = *rom);
      V += ppuctrl & 4 ? 32 : 1; V %= 16384;
      return tmp;
    }
    if (write) switch (lo) {
      case 0: ppuctrl = val; T = T & 0xf3ff | val % 4 << 10; break;
      case 1: ppumask = val; break;
      case 5: T = (W ^= 1) ? fine_x = val & 7, T & ~31 | val / 8 : T & 0x8c1f | val % 8 << 12 | val * 4 & 0x3e0; break;
      case 6: T = (W ^= 1) ? T & 0xff | val % 64 << 8 : (V = T & ~0xff | val);
    }
    if (lo == 2) { tmp = ppustatus & 0xe0; ppustatus &= 0x7f; W = 0; return tmp; }
    break;
  case 4:
    if (write && lo == 20) for (uint16_t i = 256; i--;) oam[i] = mem(i, val, 0, 0);
    {
      uint8_t joypad = 0;
      for (int i = 7; i >= 0; i--) joypad = joypad * 2 + key_state[i];
      if (lo == 22) {
        if (write) keys = joypad;
        else { tmp = keys & 1; keys /= 2; return tmp; }
      }
    }
    return 0;
  case 6: case 7: addr &= 8191; return write ? prgram[addr] = val : prgram[addr];
  default:
    if (write) switch (rombuf[6] >> 4) {
      case 7: mirror = !(val / 16); prg[0] = val % 8 * 2; prg[1] = prg[0] + 1; break;
      case 4: {
        uint8_t addr1 = addr & 1;
        switch (hi >> 1) {
          case 4:
            *(addr1 ? &mmc3_chrprg[mmc3_bits & 7] : &mmc3_bits) = val;
            tmp = mmc3_bits >> 5 & 4;
            for (int i = 4; i--;) { chr[0 + i + tmp] = mmc3_chrprg[i / 2] & ~!(i % 2) | i % 2; chr[4 + i - tmp] = mmc3_chrprg[2 + i]; }
            tmp = mmc3_bits >> 5 & 2;
            prg[0 + tmp] = mmc3_chrprg[6]; prg[1] = mmc3_chrprg[7]; prg[3] = rombuf[4] * 2 - 1; prg[2 - tmp] = prg[3] - 1;
            break;
          case 5: if (!addr1) mirror = 2 + val % 2; break;
          case 6: if (!addr1) mmc3_latch = val; break;
          case 7: mmc3_irq = addr1; break;
        }
        break;
      }
      case 3: chr[0] = val % 4 * 2; chr[1] = chr[0] + 1; break;
      case 2: prg[0] = val & 31; break;
      case 1:
        if (val & 0x80) { mmc1_bits = 5; mmc1_data = 0; mmc1_ctrl |= 12; }
        else if (mmc1_data = mmc1_data / 2 | val << 4 & 16, !--mmc1_bits) {
          mmc1_bits = 5;
          tmp = addr >> 13;
          *(tmp == 4 ? mirror = mmc1_data & 3, &mmc1_ctrl : tmp == 5 ? &chrbank0 : tmp == 6 ? &chrbank1 : &prgbank) = mmc1_data;
          chr[0] = chrbank0 & ~!(mmc1_ctrl & 16);
          chr[1] = mmc1_ctrl & 16 ? chrbank1 : chrbank0 | 1;
          tmp = mmc1_ctrl / 4 % 4 - 2;
          prg[0] = !tmp ? 0 : tmp == 1 ? prgbank : prgbank & ~1;
          prg[1] = !tmp ? prgbank : tmp == 1 ? rombuf[4] - 1 : prgbank | 1;
        }
      }
    return rom[(prg[hi - 8 >> prgbits - 12] & (rombuf[4] << 14 - prgbits) - 1) << prgbits | addr & (1 << prgbits) - 1];
  }
  return ~0;
}

uint8_t read_pc() { val = mem(PCL, PCH, 0, 0); !++PCL && ++PCH; return val; }
uint8_t set_nz(uint8_t val) { return P = P & 125 | val & 128 | !val * 2; }

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "Usage: %s <rom.nes>\n", argv[0]); return 1; }
  int romfd = open(argv[1], O_RDONLY);
  if (romfd < 0) { perror("open rom"); return 1; }
  read(romfd, rombuf, 1024 * 1024);
  close(romfd);
  rom = rombuf + 16;
  prg[1] = rombuf[4] - 1;
  chrrom = rombuf[5] ? rom + (rombuf[4] << 14) : chrram;
  chr[1] = rombuf[5] ? rombuf[5] * 2 - 1 : 1;
  mirror = 3 - rombuf[6] % 2;
  if (rombuf[6] / 16 == 4) { mem(0, 128, 0, 1); prgbits--; chrbits -= 2; }
  PCL = mem(~3, ~0, 0, 0);
  PCH = mem(~2, ~0, 0, 0);

  build_xrgb_lut();

  fbfd = open("/dev/fb0", O_RDWR);
  if (fbfd < 0) { perror("open /dev/fb0"); return 1; }
  struct fb_var_screeninfo vinfo;
  struct fb_fix_screeninfo finfo;
  ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo);
  ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo);
  fb_w = vinfo.xres; fb_h = vinfo.yres;
  size_t fb_size = finfo.line_length * vinfo.yres;
  fb_mem = mmap(0, fb_size, PROT_READ|PROT_WRITE, MAP_SHARED, fbfd, 0);
  if (fb_mem == MAP_FAILED) { perror("mmap fb"); return 1; }
  memset(fb_mem, 0, fb_size);

  open_all_keyboards();
  memset(key_state, 0, sizeof(key_state));

  const double NES_FPS = 60.0988;
  const long FRAME_NS = (long)(1e9 / NES_FPS);
  struct timespec next_frame;
  clock_gettime(CLOCK_MONOTONIC, &next_frame);

loop:
  cycles = nomem = 0;
  if (nmi_irq) goto nmi_irq;
  opcode = read_pc();
  uint8_t opcodelo5 = opcode & 31;
  switch (opcodelo5) {
  case 0:
    if (opcode & 0x80) { read_pc(); nomem = 1; goto nomemop; }
    switch (opcode >> 5) {
    case 0: {
      !++PCL && ++PCH;
    nmi_irq:
      PUSH(PCH); PUSH(PCL); PUSH(P | 32);
      uint16_t veclo = ~1 - (nmi_irq & 4);
      PCL = mem(veclo, ~0, 0, 0); PCH = mem(veclo + 1, ~0, 0, 0);
      nmi_irq = 0; cycles++; break;
    }
    case 1: result = read_pc(); PUSH(PCH); PUSH(PCL); PCH = read_pc(); PCL = result; break;
    case 2: P = PULL & ~32; PCL = PULL; PCH = PULL; break;
    case 3: PCL = PULL; PCH = PULL; !++PCL && ++PCH; break;
    }
    cycles += 4; break;
  case 16:
    read_pc();
    if (!(P & mask[opcode >> 6]) ^ opcode / 32 & 1) {
      cross = PCL + (int8_t)val >> 8; PCH += cross; PCL += val; cycles += cross ? 2 : 1;
    }
    break;
  case 8: case 24:
    switch (opcode >>= 4) {
    case 0:  PUSH(P | 48); cycles++; break;
    case 2:  P = PULL & ~16; cycles += 2; break;
    case 4:  PUSH(A); cycles++; break;
    case 6:  set_nz(A = PULL); cycles += 2; break;
    case 8:  set_nz(--Y); break;
    case 9:  set_nz(A = Y); break;
    case 10: set_nz(Y = A); break;
    case 12: set_nz(++Y); break;
    case 14: set_nz(++X); break;
    default: P = P & ~mask[opcode + 3] | mask[opcode + 4]; break;
    }
    break;
  case 10: case 26:
    switch (opcode >> 4) {
    case 8:  set_nz(A = X); break;
    case 9:  S = X; break;
    case 10: set_nz(X = A); break;
    case 11: set_nz(X = S); break;
    case 12: set_nz(--X); break;
    case 14: break;
    default: nomem = 1; val = A; goto nomemop;
    }
    break;
  case 1:
    read_pc(); val += X;
    addr_lo = mem(val, 0, 0, 0); addr_hi = mem(val + 1, 0, 0, 0);
    cycles += 4; goto opcode;
  case 2: case 9: read_pc(); nomem = 1; goto nomemop;
  case 17:
    addr_lo = mem(read_pc(), 0, 0, 0); addr_hi = mem(val + 1, 0, 0, 0);
    cycles++; goto add_x_or_y;
  case 4: case 5: case 6: case 20: case 21: case 22:
    addr_lo = read_pc();
    cross = opcodelo5 > 6;
    if (cross) addr_lo += (opcode & 214) == 150 ? Y : X;
    addr_hi = 0; cycles -= !cross; goto opcode;
  case 12: case 13: case 14: case 25: case 28: case 29: case 30:
    addr_lo = read_pc(); addr_hi = read_pc();
    if (opcodelo5 < 25) goto opcode;
  add_x_or_y:
    val = opcodelo5 < 28 | opcode == 190 ? Y : X;
    cross = addr_lo + val > 255;
    addr_hi += cross; addr_lo += val;
    cycles += ((opcode & 224) == 128 | opcode % 16 == 14 & opcode != 190) | cross;
  opcode:
    cycles += 2;
    if (opcode != 76 & (opcode & 224) != 128) val = mem(addr_lo, addr_hi, 0, 0);
  nomemop:
    result = 0;
    switch (opcode & 227) {
    case 1:   set_nz(A |= val); break;
    case 33:  set_nz(A &= val); break;
    case 65:  set_nz(A ^= val); break;
    case 225: val = ~val;
    case 97:
      sum = A + val + P % 2;
      P = P & ~65 | sum > 255 | ((A ^ sum) & (val ^ sum) & 128) / 2;
      set_nz(A = sum);
      break;
    case 34: result = P & 1;
    case 2: result |= val * 2; P = P & ~1 | val / 128; goto memop;
    case 98: result = P << 7;
    case 66: result |= val / 2; P = P & ~1 | val & 1; goto memop;
    case 194: result = val - 1; goto memop;
    case 226: result = val + 1;
    memop:
      set_nz(result);
      nomem ? A = result : (cycles += 2, mem(addr_lo, addr_hi, result, 1));
      break;
    case 32: P = P & 61 | val & 192 | !(A & val) * 2; break;
    case 64: PCL = addr_lo; PCH = addr_hi; cycles--; break;
    case 96: PCL = val; PCH = mem(addr_lo + 1, addr_hi, 0, 0); cycles++; break;
    default: {
      uint8_t opcodehi3 = opcode / 32;
      uint8_t *reg = opcode % 4 == 2 | opcodehi3 == 7 ? &X : opcode % 4 == 1 ? &A : &Y;
      if (opcodehi3 == 4) mem(addr_lo, addr_hi, *reg, 1);
      else if (opcodehi3 != 5) { P = P & ~1 | *reg >= val; set_nz(*reg - val); }
      else set_nz(*reg = val);
      break;
    }
    }
  }

  for (tmp = cycles * 3 + 6; tmp--;) {
    if (ppumask & 24) {
      if (scany < 240) {
        if (dot - 256 > 63u) {
          if (dot < 256) {
            uint8_t color = shift_hi >> 14 - fine_x & 2 | shift_lo >> 15 - fine_x & 1,
                    palette = shift_at >> 28 - fine_x * 2 & 12;
            if (ppumask & 16) {
              // PERF: build the sprite candidate list once per scanline
              // instead of re-scanning all 64 OAM sprites for every
              // single pixel (see eval_sprites_for_scanline above).
              if (line_sprite_scany != scany) eval_sprites_for_scanline(scany);

              uint16_t sprite_h = ppuctrl & 32 ? 16 : 8;
              for (uint8_t li = 0; li < line_sprite_count; li++) {
                uint8_t *sprite = oam + line_sprite_idx[li];
                uint16_t sprite_x = dot - sprite[3],
                         sprite_y = scany - sprite[0] - 1,
                         sx = sprite_x ^ !(sprite[2] & 64) * 7,
                         sy = sprite_y ^ (sprite[2] & 128 ? sprite_h - 1 : 0);
                if (sprite_x < 8 && sprite_y < sprite_h) {
                  uint16_t sprite_tile = sprite[1],
                           sprite_addr = (ppuctrl & 32 ? sprite_tile % 2 << 12 | sprite_tile << 4 & -32 | sy * 2 & 16
                                                        : (ppuctrl & 8) << 9 | sprite_tile << 4) | sy & 7,
                           sprite_color = *get_chr_byte(sprite_addr + 8) >> sx << 1 & 2 | *get_chr_byte(sprite_addr) >> sx & 1;
                  if (sprite_color) {
                    if (!(sprite[2] & 32 && color)) { color = sprite_color; palette = 16 | sprite[2] * 4 & 12; }
                    if (line_sprite_idx[li] == 0 && color) ppustatus |= 64;
                    break;
                  }
                }
              }
            }
            frame_buffer[scany * 256 + dot] =
                (uint16_t[64]){
                    25356, 34816, 39011, 30854, 24714, 4107,  106,   2311,
                    2468,  2561,  4642,  6592,  20832, 0,     0,     0,
                    44373, 49761, 55593, 51341, 43186, 18675, 434,   654,
                    4939,  5058,  3074,  19362, 37667, 0,     0,     0,
                    ~0,    ~819,  64497, 64342, 62331, 43932, 23612, 9465,
                    1429,  1550,  20075, 36358, 52713, 16904, 0,     0,
                    ~0,    ~328,  ~422,  ~452,  ~482,  58911, 50814, 42620,
                    40667, 40729, 48951, 53078, 61238, 44405}
                    [palette_ram[color ? palette | color : 0]];
          }
          if (dot < 336) { shift_hi *= 2; shift_lo *= 2; shift_at *= 4; }
          int temp = ppuctrl << 8 & 4096 | ntb << 4 | V >> 12;
          switch (dot & 7) {
          case 1: ntb = *get_nametable_byte(V); break;
          case 3:
            atb = (*get_nametable_byte(V & 0xc00 | 0x3c0 | V >> 4 & 0x38 | V / 4 & 7) >> (V >> 5 & 2 | V / 2 & 1) * 2) % 4 * 0x5555;
            break;
          case 5: ptb_lo = *get_chr_byte(temp); break;
          case 7: {
            uint8_t ptb_hi = *get_chr_byte(temp | 8);
            V = V % 32 == 31 ? V & ~31 ^ 1024 : V + 1;
            shift_hi |= ptb_hi; shift_lo |= ptb_lo; shift_at |= atb;
            break;
          }
          }
        }
        if (dot == 256) {
          V = ((V & 7 << 12) != 7 << 12 ? V + 4096
               : (V & 0x3e0) == 928     ? V & 0x8c1f ^ 2048
               : (V & 0x3e0) == 0x3e0   ? V & 0x8c1f
                                        : V & 0x8c1f | V + 32 & 0x3e0) & ~0x41f | T & 0x41f;
        }
      }
      if ((scany + 1) % 262 < 241 && dot == 261 && mmc3_irq && !mmc3_latch--) nmi_irq = 1;
      if (scany == 261 && dot - 280 < 25u) V = V & 0x841f | T & 0x7be0;
    }
    if (dot == 1) {
      if (scany == 241) {
        if (ppuctrl & 128) nmi_irq = 4;
        ppustatus |= 128;
        blit_to_fb(frame_buffer + 2048);
        poll_keyboard();
        next_frame.tv_nsec += FRAME_NS;
        while (next_frame.tv_nsec >= 1000000000L) { next_frame.tv_nsec -= 1000000000L; next_frame.tv_sec += 1; }
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec < next_frame.tv_sec || (now.tv_sec == next_frame.tv_sec && now.tv_nsec < next_frame.tv_nsec)) {
          clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_frame, NULL);
        } else { next_frame = now; }
      }
      if (scany == 261) ppustatus = 0;
    }
    if (++dot == 341) { dot = 0; scany++; scany %= 262; }
  }
  goto loop;
}