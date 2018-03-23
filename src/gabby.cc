/*
 * Copyright (C) 2018 Ben Smith
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <algorithm>
#include <array>
#include <cassert>
#include <cinttypes>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <sys/time.h>

#define DEBUG_DMA 0
#define DEBUG_INTERRUPTS 0
#define DEBUG_MODE 0
#define DEBUG_SCROLL 0
#define DEBUG_TIMER 0
#define DEBUG_WINDOW 0

#define DPRINT(x, ...) if (DEBUG_##x) { Print(__VA_ARGS__); }

#if __GNUC__ || __clang__
#define PRINTF_FORMAT(fmt, fst) __attribute__((format(printf, (fmt), (fst))))
#else
#define PRINTF_FORMAT
#endif

// Not sure about these yet.
#define DMA_DELAY 8
#define DMA_TIME 1280
#define LINE_START_TICK 8
#define MODE_START_TICK 4
#define TIMER_IF_DELAY 6
#define TIMER_RESET_DELAY 10
#define TIMER_NO_OVERFLOW (-11)

using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using f32 = float;
using f64 = double;
using Tick = s64;  // = Clock edge.
using Clock = s64;
using FClock = f64;
using Error = std::runtime_error;
using Buffer = std::vector<u8>;

constexpr Clock CpuClocksPerSec = 0x400000;
constexpr Tick MaxTick = std::numeric_limits<Tick>::max();

enum {
  JOYP = 0x00,
  SB = 0x01,
  SC = 0x02,
  DIV = 0x04,
  TIMA = 0x05,
  TMA = 0x06,
  TAC = 0x07,
  IF = 0x0f,
  NR10 = 0x10,
  NR11 = 0x11,
  NR12 = 0x12,
  NR13 = 0x13,
  NR14 = 0x14,
  NR21 = 0x16,
  NR22 = 0x17,
  NR23 = 0x18,
  NR24 = 0x19,
  NR30 = 0x1a,
  NR31 = 0x1b,
  NR32 = 0x1c,
  NR33 = 0x1d,
  NR34 = 0x1e,
  NR41 = 0x20,
  NR42 = 0x21,
  NR43 = 0x22,
  NR44 = 0x23,
  NR50 = 0x24,
  NR51 = 0x25,
  NR52 = 0x26,
  LCDC = 0x40,
  STAT = 0x41,
  SCY = 0x42,
  SCX = 0x43,
  LY = 0x44,
  LYC = 0x45,
  DMA = 0x46,
  BGP = 0x47,
  OBP0 = 0x48,
  OBP1 = 0x49,
  WY = 0x4a,
  WX = 0x4b,
  KEY1 = 0x4d,
  VBK = 0x4f,
  HDMA1 = 0x51,
  HDMA2 = 0x52,
  HDMA3 = 0x53,
  HDMA4 = 0x54,
  HDMA5 = 0x55,
  RP = 0x56,
  BCPS = 0x68,
  BCPD = 0x69,
  OCPS = 0x6a,
  OCPD = 0x6b,
  SVBK = 0x70,
  IE = 0xff,
};

enum class MBC { None, _1, _1M, _2, _3, _5, MMM01, TAMA5, HUC3, HUC1 };
enum class CGB { None, Supported, Required };
enum class ROMSize { _32k, _64k, _128k, _256k, _512k, _1m, _2m, _4m, _8m };
enum class SRAMSize { None, _2k, _8k, _32k, _128k, _64k };
enum class Variant { Guess, DMG0, DMGABCX, MGB, SGB, CGB };
enum { FZ = 0x80, FN = 0x40, FH = 0x20, FC = 0x10 };

struct Cart {
  Cart(const Buffer&, Variant);

  Variant variant;
  MBC mbc;
  CGB cgb;
  ROMSize rom_size;
  SRAMSize sram_size;
  u8 rom_bank_mask, sram_bank_mask;
  u16 sram_addr_mask;
};

struct GB;
struct State {
  explicit State(GB&);

  Tick tick, op_tick;
  union { struct { u8 f, a; }; u16 af; };
  union { struct { u8 c, b; }; u16 bc; };
  union { struct { u8 e, d; }; u16 de; };
  union { struct { u8 l, h; }; u16 hl; };
  union { struct { u8 z, w; }; u16 wz; };
  u16 sp, pc;

  u8 op, cb_op;

  Tick ppu_line_start_tick, ppu_mode_start_tick, ppu_lcd_on_tick;
  u16 ppu_map_addr, ppu_tile_addr;
  u8 ppu_mode, ppu_stall;
  u8 ppu_line_x, ppu_line_y, ppu_draw_y;
  u8 ppu_map, ppu_tile[2], ppu_next_tile[2];
  u8 ppu_buffer[160 * 144], *ppu_pixel;
  bool ppu_window;

  Tick dma_start_tick;
  bool dma_active;
  u16 dma_addr;

  Tick div_reset_tick, tima_overflow_tick;
  bool ime, ime_enable, dispatch, halt_bug;
  bool sram_enabled;
  u8 mbc_2xxx, mbc_3xxx, mbc_23xxx, mbc_45xxx, mbc_mode;

  u8 *rom0p, *rom1p, *vramp, *wram0p, *wram1p, *sramp;
  u8 vram[0x4000];  // 0x8000-0x9fff, 2 banks
  u8 sram[0x8000];  // 0xa000-0xbfff, 4 banks
  u8 wram[0x8000];  // 0xc000-0xdfff, 4 banks
  u8 oam[0x100];    // 0xfe00-0xfe9f
  u8 io[0x100];     // 0xff00-0xffff
};

struct GB {
  explicit GB(Buffer&&, Variant);
  void Step();
  void StepCPU();
  void DispatchInterrupt();
  void StepTimer();
  bool CheckDiv(u16 old_div, u16 new_div, u8 old_tac, u8 new_tac);
  void StepDMA();
  void StepPPU();
  void StepPPU_Mode2(Tick mode_tick);
  void StepPPU_Mode3(Tick mode_tick);
  void SetPPUMode(u8 mode);
  void CheckLyLyc();

  static bool rising(Tick tick) { return (tick & 1) == 0; }
  static bool falling(Tick tick) { return (tick & 1) == 1; }
  FClock clocks() const { return s.tick / 2.0; }
  static FClock clocks(Tick tick) { return tick / 2.0; }
  u16 div() const { return (s.tick - s.div_reset_tick) >> 1; }

  void set_stat_mode(u8 mode) { s.io[STAT] = (s.io[STAT] & ~3) | mode; }

  void PRINTF_FORMAT(2, 3) Print(const char* fmt, ...) const;
  int Disassemble(u16 addr, char* buffer, size_t size);
  void PrintInstruction(u16 addr);
  void Trace();

  bool UsingVRAM() const;
  bool UsingOAM() const;
  u8 Read(u16 addr);
  void Write(u16 addr, u8 val);
  void Write_ROM(u16 addr, u8 val);
  void Write_IO(u8 addr, u8 val);

  void op_done() { s.op_tick = -2; }
  static u8 zflag(u8 x) { return x ? 0 : FZ; }
  static u8 hflag(int res, int x, int y) { return ((res ^ x ^ y) << 1) & FH; }
  u8 fc_0() const { return (s.f & FC) >> 4; }
  bool f_is(u8 mask, u8 val);

  void add(u8 x, u8 c = 0);
  void adc_mr(u16 mr);
  void adc_n();
  void adc_r(u8 r);
  void add_hl_rr(u16 rr);
  void add_mr(u16 mr);
  void add_n();
  void add_r(u8 r);
  u16 add_sp(u8 x);
  void add_sp_n();
  void and_(u8 x);
  void and_mr(u16 mr);
  void and_n();
  void and_r(u8 r);
  void bit(int n, u8 r);
  void bit_r(int n, u8 r);
  void bit_mr(int n, u16 mr);
  void call_cc_nn(u8 mask, u8 val);
  void call_nn();
  void cb();
  void ccf();
  void cpl();
  void cp_mr(u16 mr);
  void cp_n();
  void cp_r(u8 r);
  void daa();
  u8 dec(u8);
  void dec_mhl();
  void dec_rr(u16& rr);
  void dec_r(u8& r);
  void di();
  void ei();
  void halt();
  u8 inc(u8);
  void inc_mhl();
  void inc_rr(u16& rr);
  void inc_r(u8& r);
  void jp_cc_nn(u8 mask, u8 val);
  void jp_hl();
  void jp_nn();
  void jr_cc_n(u8 mask, u8 val);
  void jr_n();
  void ld_a_mff00_c();
  void ld_a_mff00_n();
  void ld_a_mnn();
  void ld_a_mr(u16 mr);
  void ld_hl_sp_n();
  void ld_mff00_c_a();
  void ld_mff00_n_a();
  void ld_mhl_n();
  void ld_mnn_a();
  void ld_mnn_sp();
  void ld_mr_r(u16& mr, u8 r, int d = 0);
  void ld_r_mr(u8& r, u16& mr, int d = 0);
  void ld_r_n(u8&);
  void ld_rr_nn(u16&);
  void ld_r_r(u8& rd, u8 rs);
  void ld_sp_hl();
  void nop();
  void or_(u8 x);
  void or_mr(u16 mr);
  void or_n();
  void or_r(u8 r);
  void pop_af();
  void pop_rr(u16& rr);
  void push_rr(u16 rr);
  void res_r(int n, u8& r);
  void res_mr(int n, u16 mr);
  void ret();
  void ret_cc(u8 mask, u8 val);
  void reti();
  u8 rl(u8 x);
  u8 rlc(u8 x);
  void rla();
  void rlca();
  void rlc_r(u8& r);
  void rlc_mr(u16 mr);
  void rl_r(u8& r);
  void rl_mr(u16 mr);
  u8 rr(u8 x);
  u8 rrc(u8 x);
  void rra();
  void rrca();
  void rrc_r(u8& r);
  void rrc_mr(u16 mr);
  void rr_r(u8& r);
  void rr_mr(u16 mr);
  void rst(u8 n);
  u8 sub(u8 x, u8 c = 0);
  void sbc_mr(u16 mr);
  void sbc_n();
  void sbc_r(u8 r);
  void scf();
  void set_r(int n, u8& r);
  void set_mr(int n, u16 mr);
  u8 sla(u8 x);
  void sla_r(u8& r);
  void sla_mr(u16 mr);
  u8 sra(u8 x);
  void sra_r(u8& r);
  void sra_mr(u16 mr);
  u8 srl(u8 x);
  void srl_r(u8& r);
  void srl_mr(u16 mr);
  void stop();
  void sub_mr(u16 mr);
  void sub_n();
  void sub_r(u8 r);
  u8 swap(u8 x);
  void swap_r(u8& r);
  void swap_mr(u16 mr);
  void xor_(u8 x);
  void xor_mr(u16 mr);
  void xor_n();
  void xor_r(u8 r);

  Buffer rom;
  Cart cart;
  State s;
};

void ThrowUnless(bool test, const char* msg) {
  if (!test) {
    throw Error(msg);
  }
}

template <typename T>
void ThrowUnlessContains(T x, const std::vector<T>& v, const char* msg) {
  if (std::find(v.begin(), v.end(), x) == v.end()) {
    throw Error(msg);
  }
}

Cart::Cart(const Buffer& rom, Variant variant) {
  ThrowUnlessContains(rom.size(),
                      {1u << 15, 1u << 16, 1u << 17, 1u << 18, 1u << 19,
                       1u << 20, 1u << 21, 1u << 22, 1u << 23},
                      "Invalid ROM size.");

  ThrowUnlessContains(rom[0x143], {0, 0x80, 0xc0}, "Invalid cgb flag.");
  cgb = static_cast<CGB>(rom[0x143]);

  ThrowUnless(cgb != CGB::Required || variant == Variant::CGB ||
                  variant == Variant::Guess,
              "Variant requires CGB.");

  switch (rom[0x147]) {
    case 0: case 8: case 9: case 252: mbc = MBC::None; break;
    case 1: case 2: case 3: mbc = MBC::_1; break;
    case 5: case 6: mbc = MBC::_2; break;
    case 11: case 12: case 14: mbc = MBC::MMM01; break;
    case 15: case 16: case 17: case 18: case 19: mbc = MBC::_3; break;
    case 25: case 26: case 27: case 28: case 29: case 30: mbc = MBC::_5; break;
    case 253: mbc = MBC::TAMA5; break;
    case 254: mbc = MBC::HUC3; break;
    case 255: mbc = MBC::HUC1; break;
    default: throw Error("Invalid cart type");
  }

  ThrowUnless(rom[0x148] <= 8, "Invalid ROM size.");
  rom_size = static_cast<ROMSize>(rom[0x148]);
  rom_bank_mask = (2 << (u8)rom_size) - 1;

  ThrowUnless(rom[0x149] <= 5, "Invalid SRAM size.");
  sram_size = static_cast<SRAMSize>(rom[0x149]);
  if (mbc == MBC::_2) {
    sram_bank_mask = 0;
    sram_addr_mask = 0x1ff;
  } else {
    switch (sram_size) {
      case SRAMSize::None: sram_bank_mask = sram_addr_mask = 0; break;
      case SRAMSize::_2k: sram_bank_mask = 0; sram_addr_mask = 0x7ff; break;
      case SRAMSize::_8k: sram_bank_mask = 0; sram_addr_mask = 0x1fff; break;
      case SRAMSize::_32k: sram_bank_mask = 3; sram_addr_mask = 0x1fff; break;
      case SRAMSize::_128k: sram_bank_mask = 15; sram_addr_mask = 0x1fff; break;
      case SRAMSize::_64k: sram_bank_mask = 7; sram_addr_mask = 0x1fff; break;
    }
  }
}

State::State(GB& gb) {
  std::fill((u8*)this, (u8*)this + sizeof(State), 0);
  af = 0x01b0;
  bc = 0x0013;
  de = 0x00d8;
  hl = 0x014d;
  sp = 0xfffe;
  pc = 0x0100;
  rom0p = gb.rom.data();
  rom1p = gb.rom.data() + 0x4000;
  vramp = vram;
  wram0p = wram;
  wram1p = wram + 0x1000;
  sramp = sram;

  std::fill(oam + 0xa0, oam + 0x100, 0xff);
  std::fill(io, io + 0x80, 0xff);

  static const u8 dmg_io_init[0x4c] = {
      0xcf, 0,    0x7e, 0xff, 0xac, 0,    0,    0xf8, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xe1, 0x80, 0xbf, 0xf3, 0xff, 0xbf, 0xff,
      0x3f, 0,    0xff, 0xbf, 0x7f, 0xff, 0x9f, 0xff, 0xbf, 0xff, 0xff,
      0,    0,    0xbf, 0x77, 0xf3, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0x60, 0x0d, 0xda, 0xdd, 0x50, 0x0f, 0xad,
      0xed, 0xc0, 0xde, 0xf0, 0x0d, 0xbe, 0xef, 0xfe, 0xed, 0x91, 0x80,
      0,    0,    0,    0,    0xff, 0xfc, 0xff, 0xff, 0,    0};

  std::copy(dmg_io_init, dmg_io_init + sizeof(dmg_io_init), io);
  div_reset_tick = -0xabcc * 2;
  tima_overflow_tick = TIMER_NO_OVERFLOW;
  ppu_mode = 2;
  ppu_lcd_on_tick = 0;
  ppu_line_start_tick = -LINE_START_TICK;
  ppu_mode_start_tick = -MODE_START_TICK;
  ppu_pixel = ppu_buffer;
  dma_start_tick = MaxTick;
}

GB::GB(Buffer&& rom_, Variant variant)
    : rom(std::move(rom_)), cart(rom, variant), s(*this) {}

#define REG_OPS(code, name)              \
  case code + 0: name##_r(s.b); break;   \
  case code + 1: name##_r(s.c); break;   \
  case code + 2: name##_r(s.d); break;   \
  case code + 3: name##_r(s.e); break;   \
  case code + 4: name##_r(s.h); break;   \
  case code + 5: name##_r(s.l); break;   \
  case code + 6: name##_mr(s.hl); break; \
  case code + 7: name##_r(s.a); break;
#define REG_OPS_N(code, name, n)            \
  case code + 0: name##_r(n, s.b); break;   \
  case code + 1: name##_r(n, s.c); break;   \
  case code + 2: name##_r(n, s.d); break;   \
  case code + 3: name##_r(n, s.e); break;   \
  case code + 4: name##_r(n, s.h); break;   \
  case code + 5: name##_r(n, s.l); break;   \
  case code + 6: name##_mr(n, s.hl); break; \
  case code + 7: name##_r(n, s.a); break;
#define LD_R_OPS(code, r) REG_OPS_N(code, ld_r, r)

void GB::Step() {
  StepTimer();
  StepCPU();
  StepPPU();
  StepDMA();
  ++s.tick;
  StepPPU();
  ++s.tick;
}

void GB::StepCPU() {
  assert(rising(s.tick));
  switch (s.op_tick) {
    case 0: case 2:
      break;
    case 4:
      s.dispatch = s.ime && !!(s.io[IF] & s.io[IE] & 0x1f);
      s.ime = s.ime_enable ? true : s.ime;
      s.ime_enable = false;
      if (s.dispatch) {
        DPRINT(INTERRUPTS, "dispatch\n");
      }
      break;
    case 6:
      if (s.dispatch) { DispatchInterrupt(); break; }
      s.op = Read(s.pc++);
      if (s.halt_bug) { --s.pc; s.halt_bug = false; }
      // Fallthrough.
    default:
      if (s.dispatch) { DispatchInterrupt(); break; }

      switch (s.op) {
        case 0x00: nop(); break;
        case 0x01: ld_rr_nn(s.bc); break;
        case 0x02: ld_mr_r(s.bc, s.a); break;
        case 0x03: inc_rr(s.bc); break;
        case 0x04: inc_r(s.b); break;
        case 0x05: dec_r(s.b); break;
        case 0x06: ld_r_n(s.b); break;
        case 0x07: rlca(); break;
        case 0x08: ld_mnn_sp(); break;
        case 0x09: add_hl_rr(s.bc); break;
        case 0x0a: ld_r_mr(s.a, s.bc); break;
        case 0x0b: dec_rr(s.bc); break;
        case 0x0c: inc_r(s.c); break;
        case 0x0d: dec_r(s.c); break;
        case 0x0e: ld_r_n(s.c); break;
        case 0x0f: rrca(); break;
        case 0x10: stop(); break;
        case 0x11: ld_rr_nn(s.de); break;
        case 0x12: ld_mr_r(s.de, s.a); break;
        case 0x13: inc_rr(s.de); break;
        case 0x14: inc_r(s.d); break;
        case 0x15: dec_r(s.d); break;
        case 0x16: ld_r_n(s.d); break;
        case 0x17: rla(); break;
        case 0x18: jr_n(); break;
        case 0x19: add_hl_rr(s.de); break;
        case 0x1a: ld_r_mr(s.a, s.de); break;
        case 0x1b: dec_rr(s.de); break;
        case 0x1c: inc_r(s.e); break;
        case 0x1d: dec_r(s.e); break;
        case 0x1e: ld_r_n(s.e); break;
        case 0x1f: rra(); break;
        case 0x20: jr_cc_n(FZ, 0); break;
        case 0x21: ld_rr_nn(s.hl); break;
        case 0x22: ld_mr_r(s.hl, s.a, 1); break;
        case 0x23: inc_rr(s.hl); break;
        case 0x24: inc_r(s.h); break;
        case 0x25: dec_r(s.h); break;
        case 0x26: ld_r_n(s.h); break;
        case 0x27: daa(); break;
        case 0x28: jr_cc_n(FZ, FZ); break;
        case 0x29: add_hl_rr(s.hl); break;
        case 0x2a: ld_r_mr(s.a, s.hl, 1); break;
        case 0x2b: dec_rr(s.hl); break;
        case 0x2c: inc_r(s.l); break;
        case 0x2d: dec_r(s.l); break;
        case 0x2e: ld_r_n(s.l); break;
        case 0x2f: cpl(); break;
        case 0x30: jr_cc_n(FC, 0); break;
        case 0x31: ld_rr_nn(s.sp); break;
        case 0x32: ld_mr_r(s.hl, s.a, -1); break;
        case 0x33: inc_rr(s.sp); break;
        case 0x34: inc_mhl(); break;
        case 0x35: dec_mhl(); break;
        case 0x36: ld_mhl_n(); break;
        case 0x37: scf(); break;
        case 0x38: jr_cc_n(FC, FC); break;
        case 0x39: add_hl_rr(s.sp); break;
        case 0x3a: ld_r_mr(s.a, s.hl, -1); break;
        case 0x3b: dec_rr(s.sp); break;
        case 0x3c: inc_r(s.a); break;
        case 0x3d: dec_r(s.a); break;
        case 0x3e: ld_r_n(s.a); break;
        case 0x3f: ccf(); break;
        LD_R_OPS(0x40, s.b)
        LD_R_OPS(0x48, s.c)
        LD_R_OPS(0x50, s.d)
        LD_R_OPS(0x58, s.e)
        LD_R_OPS(0x60, s.h)
        LD_R_OPS(0x68, s.l)
        case 0x70: ld_mr_r(s.hl, s.b); break;
        case 0x71: ld_mr_r(s.hl, s.c); break;
        case 0x72: ld_mr_r(s.hl, s.d); break;
        case 0x73: ld_mr_r(s.hl, s.e); break;
        case 0x74: ld_mr_r(s.hl, s.h); break;
        case 0x75: ld_mr_r(s.hl, s.l); break;
        case 0x76: halt(); break;
        case 0x77: ld_mr_r(s.hl, s.a); break;
        LD_R_OPS(0x78, s.a)
        REG_OPS(0x80, add)
        REG_OPS(0x88, adc)
        REG_OPS(0x90, sub)
        REG_OPS(0x98, sbc)
        REG_OPS(0xa0, and)
        REG_OPS(0xa8, xor)
        REG_OPS(0xb0, or)
        REG_OPS(0xb8, cp)
        case 0xc0: ret_cc(FZ, 0); break;
        case 0xc1: pop_rr(s.bc); break;
        case 0xc2: jp_cc_nn(FZ, 0); break;
        case 0xc3: jp_nn(); break;
        case 0xc4: call_cc_nn(FZ, 0); break;
        case 0xc5: push_rr(s.bc); break;
        case 0xc6: add_n(); break;
        case 0xc7: rst(0x00); break;
        case 0xc8: ret_cc(FZ, FZ); break;
        case 0xc9: ret(); break;
        case 0xca: jp_cc_nn(FZ, FZ); break;
        case 0xcb: cb(); break;
        case 0xcc: call_cc_nn(FZ, FZ); break;
        case 0xcd: call_nn(); break;
        case 0xce: adc_n(); break;
        case 0xcf: rst(0x08); break;
        case 0xd0: ret_cc(FC, 0); break;
        case 0xd1: pop_rr(s.de); break;
        case 0xd2: jp_cc_nn(FC, 0); break;
        case 0xd4: call_cc_nn(FC, 0); break;
        case 0xd5: push_rr(s.de); break;
        case 0xd6: sub_n(); break;
        case 0xd7: rst(FC); break;
        case 0xd8: ret_cc(FC, FC); break;
        case 0xd9: reti(); break;
        case 0xda: jp_cc_nn(FC, FC); break;
        case 0xdc: call_cc_nn(FC, FC); break;
        case 0xde: sbc_n(); break;
        case 0xdf: rst(0x18); break;
        case 0xe0: ld_mff00_n_a(); break;
        case 0xe1: pop_rr(s.hl); break;
        case 0xe2: ld_mff00_c_a(); break;
        case 0xe5: push_rr(s.hl); break;
        case 0xe6: and_n(); break;
        case 0xe7: rst(0x20); break;
        case 0xe8: add_sp_n(); break;
        case 0xe9: jp_hl(); break;
        case 0xea: ld_mnn_a(); break;
        case 0xee: xor_n(); break;
        case 0xef: rst(0x28); break;
        case 0xf0: ld_a_mff00_n(); break;
        case 0xf1: pop_af(); break;
        case 0xf2: ld_a_mff00_c(); break;
        case 0xf3: di(); break;
        case 0xf5: push_rr(s.af); break;
        case 0xf6: or_n(); break;
        case 0xf7: rst(0x30); break;
        case 0xf8: ld_hl_sp_n(); break;
        case 0xf9: ld_sp_hl(); break;
        case 0xfa: ld_a_mnn(); break;
        case 0xfb: ei(); break;
        case 0xfe: cp_n(); break;
        case 0xff: rst(0x38); break;
      }
      break;
  }
  s.op_tick += 2;
}

void GB::cb() {
  switch (s.op_tick) {
    case 6: case 8: case 10: case 12:
      break;
    case 14:
      s.cb_op = Read(s.pc++);
      // Fallthrough.
    default:
      switch (s.cb_op) {
        REG_OPS(0x00, rlc)
        REG_OPS(0x08, rrc)
        REG_OPS(0x10, rl)
        REG_OPS(0x18, rr)
        REG_OPS(0x20, sla)
        REG_OPS(0x28, sra)
        REG_OPS(0x30, swap)
        REG_OPS(0x38, srl)
        REG_OPS_N(0x40, bit, 0)
        REG_OPS_N(0x48, bit, 1)
        REG_OPS_N(0x50, bit, 2)
        REG_OPS_N(0x58, bit, 3)
        REG_OPS_N(0x60, bit, 4)
        REG_OPS_N(0x68, bit, 5)
        REG_OPS_N(0x70, bit, 6)
        REG_OPS_N(0x78, bit, 7)
        REG_OPS_N(0x80, res, 0)
        REG_OPS_N(0x88, res, 1)
        REG_OPS_N(0x90, res, 2)
        REG_OPS_N(0x98, res, 3)
        REG_OPS_N(0xa0, res, 4)
        REG_OPS_N(0xa8, res, 5)
        REG_OPS_N(0xb0, res, 6)
        REG_OPS_N(0xb8, res, 7)
        REG_OPS_N(0xc0, set, 0)
        REG_OPS_N(0xc8, set, 1)
        REG_OPS_N(0xd0, set, 2)
        REG_OPS_N(0xd8, set, 3)
        REG_OPS_N(0xe0, set, 4)
        REG_OPS_N(0xe8, set, 5)
        REG_OPS_N(0xf0, set, 6)
        REG_OPS_N(0xf8, set, 7)
      }
      break;
  }
}

#undef REG_OPS
#undef REG_OPS_N
#undef LD_R_OPS

void GB::DispatchInterrupt() {
  switch (s.op_tick) {
    case 14: {
      Write(--s.sp, s.pc >> 8);
      u8 intr = s.io[IF] & s.io[IE] & 0x1f;
      s.wz = 0;
      for (int i = 0; i < 5; ++i) {
        if (intr & (1 << i)) {
          s.io[IF] &= ~(1 << i);
          s.wz = 0x40 + (i << 3);
          DPRINT(INTERRUPTS, "dispatch to interrupt %d @ %04x\n", 1 << i, s.wz);
          break;
        }
      }
      break;
    }
    case 22: Write(--s.sp, s.pc); break;
    case 38: s.pc = s.wz; s.ime = false; op_done(); break;
  }
}

bool GB::UsingVRAM() const { return s.ppu_mode == 3; }
bool GB::UsingOAM() const { return s.dma_active || s.ppu_mode == 2; }

u8 GB::Read(u16 addr) {
  switch (addr >> 12) {
    case 0: case 1: case 2: case 3: return s.rom0p[addr & 0x3fff];
    case 4: case 5: case 6: case 7: return s.rom1p[addr & 0x3fff];
    case 8: case 9: return UsingVRAM() ? 0xff : s.vramp[addr & 0x1fff];
    case 10: case 11:
      return s.sram_enabled ? s.sramp[addr & cart.sram_addr_mask & 0x1fff]
                            : 0xff;
    case 12: case 14: return s.wram0p[addr & 0xfff];
    case 13: return s.wram1p[addr & 0xfff];
    case 15:
      switch ((addr >> 8) & 0xf) {
        default: return s.wram1p[addr & 0xfff];
        case 14: return UsingOAM() ? 0xff : s.oam[addr & 0xff];
        case 15: return s.io[addr & 0xff];
      }
  }
  return 0xff;
}

void GB::Write(u16 addr, u8 val) {
  switch (addr >> 12) {
    case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7:
      Write_ROM(addr & 0x7fff, val);
      break;
    case 8: case 9: if (!UsingVRAM()) { s.vramp[addr & 0x1fff] = val; } break;
    case 10: case 11:
      if (s.sram_enabled) {
        s.sramp[addr & cart.sram_addr_mask & 0x1fff] = val;
      }
      break;
    case 12: case 14: s.wram0p[addr & 0xfff] = val; break;
    case 13: s.wram1p[addr & 0xfff] = val; break;
    case 15:
      switch ((addr >> 8) & 0xf) {
        default: s.wram1p[addr & 0xfff] = val; break;
        case 14: if (!UsingOAM()) { s.oam[addr & 0xff] = val; } break;
        case 15: Write_IO(addr & 0xff, val); break;
      }
  }
}

void GB::Write_ROM(u16 addr, u8 val) {
  switch (addr >> 12) {
    case 0: case 1: s.sram_enabled = (val & 0xf) == 0xa; break;
    case 2: s.mbc_2xxx = val; s.mbc_23xxx = val; break;
    case 3: s.mbc_3xxx = val; s.mbc_23xxx = val; break;
    case 4: case 5: s.mbc_45xxx = val; break;
    case 6: case 7: s.mbc_mode = val; break;
  }

  u8 rom0_bank = 0, rom1_bank = 1, sram_bank = 0;
  switch (cart.mbc) {
    {
      u8 mask, shift;
      case MBC::_1: mask = 0x1f; shift = 5; goto mbc1_shared;
      case MBC::_1M: mask = 0xf; shift = 4; goto mbc1_shared;
      mbc1_shared:
        u8 hi_bank = (s.mbc_45xxx & 3) << shift;
        if (s.mbc_mode) {
          rom0_bank |= hi_bank;
          sram_bank = (s.mbc_45xxx & 3);
        }
        rom1_bank = s.mbc_23xxx & mask;
        if (rom1_bank == 0) {
          ++rom1_bank;
        }
        rom1_bank |= hi_bank;
        break;
    }
    case MBC::_2:
      rom1_bank = s.mbc_23xxx & 0xf;
      break;
    case MBC::_3:
      rom1_bank = s.mbc_23xxx & 0x7f;
      sram_bank = s.mbc_45xxx & 0x7;
      break;
    case MBC::_5:
      rom1_bank = ((s.mbc_3xxx & 1) << 8) | s.mbc_2xxx;
      sram_bank = s.mbc_45xxx & 0xf;
      break;
    case MBC::HUC1: {
      rom1_bank = s.mbc_23xxx & 0x3f;
      if (rom1_bank == 0) { rom1_bank++; }
      if (s.mbc_mode) {
        sram_bank = s.mbc_45xxx;
      } else {
        rom1_bank |= (s.mbc_45xxx & 3) << 6;
      }
      break;
    }
    case MBC::MMM01: break;  // TODO
    default: break;
  }
  s.rom0p = rom.data() + ((rom0_bank & cart.rom_bank_mask) << 14);
  s.rom1p = rom.data() + ((rom1_bank & cart.rom_bank_mask) << 14);
  s.sramp = s.sram + ((sram_bank & cart.sram_bank_mask) << 13);
}

void GB::Write_IO(u8 addr, u8 val) {
  static const u8 dmg_io_mask[256] = {
      0x30, 0xff, 0x81, 0,    0,    0xff, 0xff, 0x07, 0,    0,    0,    0,
      0,    0,    0,    0x1f, 0x7f, 0xff, 0xff, 0xff, 0x40, 0,    0xff, 0xff,
      0xff, 0x40, 0x80, 0xff, 0x60, 0xff, 0x40, 0,    0,    0xff, 0xff, 0x40,
      0xff, 0xff, 0x80, 0,    0,    0,    0,    0,    0,    0,    0,    0,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0x78, 0xff, 0xff, 0,    0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff};

  u8& byte = s.io[addr];
  u8 old = byte;
  u8 mask = dmg_io_mask[addr];
  byte = (byte & ~mask) | (val & mask);

  switch (addr) {
    case DIV: {
      DPRINT(TIMER, "wrote div\n");
      u16 old_div = div();
      s.div_reset_tick = s.tick;
      s.io[DIV] = 0;
      if (CheckDiv(old_div, 0, s.io[TAC], s.io[TAC])) {
      DPRINT(TIMER, "++tima from div write\n");
      }
      break;
    }
    case TIMA:
      DPRINT(TIMER, "wrote tima=%02x->%02x\n", old, byte);
      // TODO(binji): gotta be a simpler solution here...
      if (s.tick - s.tima_overflow_tick == TIMER_RESET_DELAY) {
        DPRINT(TIMER, "  overriden by TIMA reset =>%02x\n", old);
        byte = old;
      }
      break;
    case TMA:
      DPRINT(TIMER, "wrote tma=%02x->%02x\n", old, byte);
      // TODO(binji): gotta be a simpler solution here...
      if (s.tick - s.tima_overflow_tick == TIMER_RESET_DELAY) {
        DPRINT(TIMER, "  copied to TIMA\n");
        s.io[TIMA] = byte;
      }
      break;
    case TAC:
      DPRINT(TIMER, "wrote tac=%02x->%02x enable=%d clock=%d\n", old, byte,
            (byte & 4) >> 2, byte & 3);
      if (!(old & 4)) {
        u16 cur_div = div();
        if (CheckDiv(cur_div, (byte & 4) ? 0 : cur_div, old, byte)) {
          DPRINT(TIMER, "++tima from tac write\n");
        }
      }
      break;
    case LCDC:
      if ((old ^ byte) & 0x80) {
        bool enabled = !!(byte & 0x80);
        if (enabled) {
          s.ppu_lcd_on_tick = s.tick + 1;
          s.ppu_line_start_tick = s.ppu_lcd_on_tick - LINE_START_TICK;
          s.ppu_mode_start_tick = s.ppu_lcd_on_tick - MODE_START_TICK;
          s.ppu_line_x = s.ppu_line_y = 0;
          s.ppu_pixel = s.ppu_buffer;
          DPRINT(MODE, "(%ld) lcd on\n", s.tick - s.ppu_lcd_on_tick);
        } else {
          s.ppu_lcd_on_tick = MaxTick;
        }
        s.ppu_mode = enabled ? 2 : 0;
        s.io[LY] = 0;
        s.io[STAT] &= ~7;
        CheckLyLyc();
      }
      break;
    case DMA:
      s.dma_start_tick = s.tick + DMA_DELAY;
      s.dma_addr = val << 8;
      DPRINT(DMA, "dma triggered, active=%.f\n", clocks(s.dma_start_tick));
      break;

    case WX:
      DPRINT(WINDOW, "wx: %d ly: %d\n", s.io[WX], s.io[LY]);
      break;

    case WY:
      DPRINT(WINDOW, "wy: %d ly: %d\n", s.io[WY], s.io[LY]);
      break;

    case SCX:
      DPRINT(SCROLL, "scx: %d ly: %d\n", s.io[SCX], s.io[LY]);
      break;

    case SCY:
      DPRINT(SCROLL, "scy: %d ly: %d\n", s.io[SCY], s.io[LY]);
      break;
  }
}

bool GB::f_is(u8 mask, u8 val) {
  if ((s.f & mask) == val) {
    return true;
  }
  op_done();
  return false;
}

void GB::add(u8 x, u8 c) {
  int r = s.a + x + c;
  s.f = zflag(r) | hflag(r, s.a, x) | ((r >> 4) & FC);
  s.a = r;
}

void GB::adc_mr(u16 mr) {
  switch (s.op_tick) {
    case 14: add(Read(mr), fc_0()); op_done(); break;
  }
}

void GB::adc_n() {
  switch (s.op_tick) {
    case 14: add(Read(s.pc++), fc_0()); op_done(); break;
  }
}

void GB::adc_r(u8 r) {
  add(r, fc_0());
  op_done();
}

void GB::add_hl_rr(u16 rr) {
  switch (s.op_tick) {
    case 14: {
      int res = s.hl + rr;
      s.f = (s.f & FZ) | (((s.hl ^ rr ^ res) >> 7) & FH) | ((res >> 12) & FC);
      s.hl = res;
      op_done();
      break;
    }
  }
}

void GB::add_mr(u16 mr) {
  switch (s.op_tick) {
    case 14: add(Read(mr)); op_done(); break;
  }
}

void GB::add_n() {
  switch (s.op_tick) {
    case 14: add(Read(s.pc++)); op_done(); break;
  }
}

void GB::add_r(u8 r) {
  add(r);
  op_done();
}

u16 GB::add_sp(u8 x) {
  int res = (u8)s.sp + x;
  s.f = hflag(res, (u8)s.sp, x) | ((res >> 4) & FC);
  return s.sp + (s8)x;
}

void GB::add_sp_n() {
  switch (s.op_tick) {
    case 14: s.sp = add_sp(Read(s.pc++)); break;
    case 30: op_done(); break;
  }
}

void GB::and_(u8 x) {
  s.a &= x;
  s.f = zflag(s.a) | FH;
}

void GB::and_mr(u16 mr) {
  switch (s.op_tick) {
    case 14: and_(Read(mr)); op_done(); break;
  }
}

void GB::and_n() {
  switch (s.op_tick) {
    case 14: and_(Read(s.pc++)); op_done(); break;
  }
}

void GB::and_r(u8 r) {
  and_(r);
  op_done();
}

void GB::bit(int n, u8 r) { s.f = (s.f & FC) | zflag(r & (1 << n)) | FH; }

void GB::bit_r(int n, u8 r) {
  switch (s.op_tick) {
    case 14: bit(n, r); op_done(); break;
  }
}

void GB::bit_mr(int n, u16 mr) {
  switch (s.op_tick) {
    case 22: bit(n, Read(mr)); op_done(); break;
  }
}

void GB::call_cc_nn(u8 mask, u8 val) {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: s.w = Read(s.pc++); f_is(mask, val); break;
    case 38: Write(--s.sp, s.pc >> 8); break;
    case 46: Write(--s.sp, s.pc); s.pc = s.wz; op_done(); break;
  }
}

void GB::call_nn() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: s.w = Read(s.pc++); break;
    case 38: Write(--s.sp, s.pc >> 8); break;
    case 46: Write(--s.sp, s.pc); s.pc = s.wz; op_done(); break;
  }
}

void GB::ccf() {
  s.f = (s.f & FZ) | ((s.f ^ FC) & FC);
  op_done();
}

void GB::cpl() {
  s.a = ~s.a;
  s.f = (s.f & (FZ | FC)) | (FN | FH);
  op_done();
}

void GB::cp_mr(u16 mr) {
  switch (s.op_tick) {
    case 14: sub(Read(mr)); op_done(); break;
  }
}

void GB::cp_n() {
  switch (s.op_tick) {
    case 14: sub(Read(s.pc++)); op_done(); break;
  }
}

void GB::cp_r(u8 r) {
  sub(r);
  op_done();
}

void GB::daa() {
  u8 r = 0;
  if ((s.f & FH) || (!(s.f & FN) && (s.a & 0xf) > 9)) {
    r = 6;
  }
  if ((s.f & FC) || (!(s.f & FN) && s.a > 0x99)) {
    r |= 0x60;
    s.f |= FC;
  }
  s.a += (s.f & FN) ? -r : r;
  s.f = (s.f & (FN | FC)) | (s.a ? 0 : FZ);
  op_done();
}

u8 GB::dec(u8 r) {
  int res = r - 1;
  s.f = (s.f & FC) | zflag(res) | FN | hflag(res, r, 0);
  return res;
}

void GB::dec_mhl() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.hl); break;
    case 22: Write(s.hl, dec(s.z)); op_done(); break;
  }
}

void GB::dec_rr(u16& rr) {
  switch (s.op_tick) {
    case 6: rr--; break;
    case 14: op_done(); break;
  }
}

void GB::dec_r(u8& r) {
  r = dec(r);
  op_done();
}

void GB::di() {
  DPRINT(INTERRUPTS, "di\n");
  s.ime_enable = s.ime = false;
  op_done();
}

void GB::ei() {
  DPRINT(INTERRUPTS, "ei\n");
  s.ime_enable = true;
  op_done();
}

void GB::halt() {
  if (s.op_tick == 6 || (s.op_tick & 7) == 0) {
    if (s.io[IF] & s.io[IE] & 0x1f) {
      s.dispatch = s.ime;
      if (!s.ime && s.op_tick == 6) { s.halt_bug = true; }
      s.op_tick &= 7;
    }
  }
}

u8 GB::inc(u8 r) {
  int res = r + 1;
  s.f = (s.f & FC) | zflag(res) | hflag(res, r, 0);
  return res;
}

void GB::inc_mhl() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.hl); break;
    case 22: Write(s.hl, inc(s.z)); op_done(); break;
  }
}

void GB::inc_rr(u16& rr) {
  switch (s.op_tick) {
    case 6: rr++; break;
    case 14: op_done(); break;
  }
}

void GB::inc_r(u8& r) {
  r = inc(r);
  op_done();
}

void GB::jp_cc_nn(u8 mask, u8 val) {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: s.w = Read(s.pc++); if (f_is(mask, val)) { s.pc = s.wz; } break;
    case 30: op_done(); break;
  }
}

void GB::jp_hl() {
  s.pc = s.hl;
  op_done();
}

void GB::jp_nn() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: s.w = Read(s.pc++); s.pc = s.wz; break;
    case 30: s.pc = s.wz; op_done(); break;
  }
}

void GB::jr_cc_n(u8 mask, u8 val) {
  switch (s.op_tick) {
    case 14:
      s.z = Read(s.pc++);
      if (f_is(mask, val)) {
        s.pc += (s8)s.z;
      }
      break;
    case 22: op_done(); break;
  }
}

void GB::jr_n() {
  switch (s.op_tick) {
    case 14: s.pc += (s8)Read(s.pc++); break;
    case 22: op_done(); break;
  }
}

void GB::ld_a_mff00_c() {
  switch (s.op_tick) {
    case 14: s.a = Read(0xff00 + s.c); op_done(); break;
  }
}

void GB::ld_a_mff00_n() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: s.a = Read(0xff00 + s.z); op_done(); break;
  }
}

void GB::ld_a_mnn() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: s.w = Read(s.pc++); break;
    case 30: s.a = Read(s.wz); op_done(); break;
  }
}

void GB::ld_a_mr(u16 mr) {
  switch (s.op_tick) {
    case 14: s.a = Read(mr); op_done(); break;
  }
}

void GB::ld_hl_sp_n() {
  switch (s.op_tick) {
    case 14: s.hl = add_sp(Read(s.pc++)); break;
    case 22: op_done(); break;
  }
}

void GB::ld_mff00_c_a() {
  switch (s.op_tick) {
    case 14: Write(0xff00 + s.c, s.a); op_done(); break;
  }
}

void GB::ld_mff00_n_a() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: Write(0xff00 + s.z, s.a); op_done(); break;
  }
}

void GB::ld_mhl_n() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: Write(s.hl, s.z); op_done(); break;
  }
}

void GB::ld_mnn_a() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: s.w = Read(s.pc++); break;
    case 30: Write(s.wz, s.a); op_done(); break;
  }
}

void GB::ld_mnn_sp() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: s.w = Read(s.pc++); break;
    case 30: Write(s.wz + 1, s.sp >> 8); break;
    case 38: Write(s.wz, s.sp); op_done(); break;
  }
}

void GB::ld_mr_r(u16& mr, u8 r, int d) {
  switch (s.op_tick) {
    case 14: Write(mr, r); mr += d; op_done(); break;
  }
}

void GB::ld_r_mr(u8& r, u16& mr, int d) {
  switch (s.op_tick) {
    case 14: r = Read(mr); mr += d; op_done(); break;
  }
}

void GB::ld_r_n(u8& r) {
  switch (s.op_tick) {
    case 14: r = Read(s.pc++); op_done(); break;
  }
}

void GB::ld_rr_nn(u16& rr) {
  switch (s.op_tick) {
    case 14: s.z = Read(s.pc++); break;
    case 22: s.w = Read(s.pc++); rr = s.wz; op_done(); break;
  }
}

void GB::ld_r_r(u8& rd, u8 rs) {
  rd = rs;
  op_done();
}

void GB::ld_sp_hl() {
  switch (s.op_tick) {
    case 6: s.sp = s.hl; break;
    case 14: op_done(); break;
  }
}

void GB::nop() { op_done(); }

void GB::or_(u8 x) {
  s.a |= x;
  s.f = zflag(s.a);
}

void GB::or_mr(u16 mr) {
  switch (s.op_tick) {
    case 14: or_(Read(mr)); op_done(); break;
  }
}

void GB::or_n() {
  switch (s.op_tick) {
    case 14: or_(Read(s.pc++)); op_done(); break;
  }
}

void GB::or_r(u8 r) {
  or_(r);
  op_done();
}

void GB::pop_af() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.sp++); break;
    case 22: s.w = Read(s.sp++); s.af = s.wz & 0xfff0; op_done(); break;
  }
}

void GB::pop_rr(u16& rr) {
  switch (s.op_tick) {
    case 14: s.z = Read(s.sp++); break;
    case 22: s.w = Read(s.sp++); rr = s.wz; op_done(); break;
  }
}

void GB::push_rr(u16 rr) {
  switch (s.op_tick) {
    case 22: Write(--s.sp, rr >> 8); break;
    case 30: Write(--s.sp, rr); op_done(); break;
  }
}

void GB::res_r(int n, u8& r) {
  r &= ~(1 << n);
  op_done();
}

void GB::res_mr(int n, u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, s.z & ~(1 << n)); op_done(); break;
  }
}

void GB::ret() {
  switch (s.op_tick) {
    case 14: s.z = Read(s.sp++); break;
    case 22: s.w = Read(s.sp++); break;
    case 30: s.pc = s.wz; op_done(); break;
  }
}

void GB::ret_cc(u8 mask, u8 val) {
  switch (s.op_tick) {
    case 14: f_is(mask, val); break;
    case 22: s.z = Read(s.sp++); break;
    case 30: s.w = Read(s.sp++); break;
    case 38: s.pc = s.wz; op_done(); break;
  }
}

void GB::reti() {
  switch (s.op_tick) {
    case 6: s.ime_enable = false; s.ime = true; break;
    case 14: s.z = Read(s.sp++); break;
    case 22: s.w = Read(s.sp++); break;
    case 30: s.pc = s.wz; op_done(); break;
  }
}

u8 GB::rl(u8 x) {
  u8 c = (x >> 3) & FC;
  u8 r = (x << 1) | fc_0();
  s.f = zflag(r) | c;
  return r;
}

u8 GB::rlc(u8 x) {
  u8 c = (x >> 3) & FC;
  u8 r = (x << 1) | (c >> 4);
  s.f = zflag(r) | c;
  return r;
}

void GB::rla() {
  s.a = rl(s.a);
  s.f &= ~FZ;
  op_done();
}

void GB::rlca() {
  s.a = rlc(s.a);
  s.f &= ~FZ;
  op_done();
}

void GB::rlc_r(u8& r) {
  r = rlc(r);
  op_done();
}

void GB::rlc_mr(u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, rlc(s.z)); op_done(); break;
  }
}

void GB::rl_r(u8& r) {
  r = rl(r);
  op_done();
}

void GB::rl_mr(u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, rl(s.z)); op_done(); break;
  }
}

u8 GB::rr(u8 x) {
  u8 c = (x << 4) & FC;
  u8 r = ((s.f & FC) << 3) | (x >> 1);
  s.f = zflag(r) | c;
  return r;
}

u8 GB::rrc(u8 x) {
  u8 c = (x << 4) & FC;
  u8 r = (c << 3) | (x >> 1);
  s.f = zflag(r) | c;
  return r;
}

void GB::rra() {
  s.a = rr(s.a);
  s.f &= ~FZ;
  op_done();
}

void GB::rrca() {
  s.a = rrc(s.a);
  s.f &= ~FZ;
  op_done();
}

void GB::rrc_r(u8& r) {
  r = rrc(r);
  op_done();
}

void GB::rrc_mr(u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, rrc(s.z)); op_done(); break;
  }
}

void GB::rr_r(u8& r) {
  r = rr(r);
  op_done();
}

void GB::rr_mr(u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, rr(s.z)); op_done(); break;
  }
}

void GB::rst(u8 n) {
  switch (s.op_tick) {
    case 22: Write(--s.sp, s.pc >> 8); break;
    case 30: Write(--s.sp, s.pc); s.pc = n; op_done(); break;
  }
}

u8 GB::sub(u8 x, u8 c) {
  int r = s.a - x - c;
  s.f = zflag(r) | FN | hflag(r, s.a, x) | ((r >> 4) & FC);
  return r;
}

void GB::sbc_mr(u16 mr) {
  switch (s.op_tick) {
    case 14: s.a = sub(Read(mr), fc_0()); op_done(); break;
  }
}

void GB::sbc_n() {
  switch (s.op_tick) {
    case 14: s.a = sub(Read(s.pc++), fc_0()); op_done(); break;
  }
}

void GB::sbc_r(u8 r) {
  s.a = sub(r, fc_0());
  op_done();
}

void GB::scf() {
  s.f = (s.f & FZ) | FC;
  op_done();
}

void GB::set_r(int n, u8& r) {
  r |= (1 << n);
  op_done();
}

void GB::set_mr(int n, u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, s.z | (1 << n)); op_done(); break;
  }
}

u8 GB::sla(u8 x) {
  u8 c = (x >> 3) & FC;
  u8 r = x << 1;
  s.f = zflag(r) | c;
  return r;
}

void GB::sla_r(u8& r) {
  r = sla(r);
  op_done();
}

void GB::sla_mr(u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, sla(s.z)); op_done(); break;
  }
}

u8 GB::sra(u8 x) {
  u8 c = (x << 4) & FC;
  u8 r = (u8)(s8(x) >> 1);
  s.f = zflag(r) | c;
  return r;
}

void GB::sra_r(u8& r) {
  r = sra(r);
  op_done();
}

void GB::sra_mr(u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, sra(s.z)); op_done(); break;
  }
}

u8 GB::srl(u8 x) {
  u8 c = (x << 4) & FC;
  u8 r = x >> 1;
  s.f = zflag(r) | c;
  return r;
}

void GB::srl_r(u8& r) {
  r = srl(r);
  op_done();
}

void GB::srl_mr(u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, srl(s.z)); op_done(); break;
  }
}

void GB::stop() {}

void GB::sub_mr(u16 mr) {
  switch (s.op_tick) {
    case 14: s.a = sub(Read(mr), 0); op_done(); break;
  }
}

void GB::sub_n() {
  switch (s.op_tick) {
    case 14: s.a = sub(Read(s.pc++), 0); op_done(); break;
  }
}

void GB::sub_r(u8 r) {
  s.a = sub(r, 0);
  op_done();
}

u8 GB::swap(u8 x) {
  u8 r = (x << 4) | (x >> 4);
  s.f = zflag(r);
  return r;
}

void GB::swap_r(u8& r) {
  r = swap(r);
  op_done();
}

void GB::swap_mr(u16 mr) {
  switch (s.op_tick) {
    case 22: s.z = Read(mr); break;
    case 30: Write(mr, swap(s.z)); op_done(); break;
  }
}

void GB::xor_(u8 x) {
  s.a ^= x;
  s.f = zflag(s.a);
}

void GB::xor_mr(u16 mr) {
  switch (s.op_tick) {
    case 14: xor_(Read(mr)); op_done(); break;
  }
}

void GB::xor_n() {
  switch (s.op_tick) {
    case 14: xor_(Read(s.pc++)); op_done(); break;
  }
}

void GB::xor_r(u8 r) {
  xor_(r);
  op_done();
}

void GB::StepTimer() {
  u16 new_div = div() + 1;
  u8 tac = s.io[TAC];
  s.io[DIV] = new_div >> 8;

  if (!(tac & 4)) {
    return;
  }

  // TODO(binji): Does this happen even if the timer is subsequently disabled?
  switch (s.tick - s.tima_overflow_tick) {
    case TIMER_IF_DELAY:
      DPRINT(TIMER, "check TIMA=%02x\n", s.io[TIMA]);
      if (s.io[TIMA] == 0) {
        s.io[IF] |= 4;
        DPRINT(INTERRUPTS, "trigger TIMER IF 4\n");
      } else {
        s.tima_overflow_tick = TIMER_NO_OVERFLOW;
      }
      break;
    case TIMER_RESET_DELAY:
      s.io[TIMA] = s.io[TMA];
      DPRINT(TIMER, "reset TIMA %02x->%02x\n", s.io[TIMA], s.io[TMA]);
      break;
  }

  CheckDiv(new_div - 1, new_div, tac, tac);
}

bool GB::CheckDiv(u16 old_div, u16 new_div, u8 old_tac, u8 new_tac) {
  static const u16 tima_mask[] = {1 << 9, 1 << 3, 1 << 5, 1 << 7};
  bool old_bit = old_div & tima_mask[old_tac & 3];
  bool bit = new_div & tima_mask[new_tac & 3];
  if (old_bit != bit && !bit) {
    DPRINT(TIMER, "++tima: %02x->%02x\n", s.io[TIMA], (s.io[TIMA] + 1) & 0xff);
    if (++s.io[TIMA] == 0) {
      DPRINT(TIMER, "tima overflow, div:%04x->%04x tac:%02x->%02x\n", old_div,
             new_div, old_tac, new_tac);
      s.tima_overflow_tick = s.tick;
    }
    return true;
  }
  return false;
}

void GB::StepDMA() {
  if (s.tick >= s.dma_start_tick) {
    s.dma_active = true;
    Tick delta = s.tick - s.dma_start_tick;
    if (delta >= DMA_TIME) {
      DPRINT(DMA, "dma finished\n");
      s.dma_start_tick = MaxTick;
      s.dma_active = false;
      return;
    }

    if ((s.tick & 7) == 6) {
      u16 offset = delta >> 3;
      if (offset < 160) {
        s.oam[offset] = Read(s.dma_addr + offset);
      }
    }
  }
}

void GB::StepPPU() {
  if (s.tick < s.ppu_lcd_on_tick) { return; }

  u8& ly = s.io[LY];

  Tick mode_tick = s.tick - s.ppu_mode_start_tick;
  Tick line_tick = s.tick - s.ppu_line_start_tick;
  switch (s.ppu_mode) {
    case 0:
    case 1:
      if (line_tick == 4 && s.ppu_line_y == 144) {
        SetPPUMode(1);
      } else if (line_tick == 9 && s.ppu_line_y == 153) {
        ly = 0;
        CheckLyLyc();
      } else if (line_tick == 909 && s.ppu_line_y != 153) {
        ++ly;
        CheckLyLyc();
      } else if (line_tick == 911) {
        ++s.ppu_line_y;
        if (s.ppu_line_y < 144) {
          SetPPUMode(2);
        } else if (s.ppu_line_y == 154) {
          s.ppu_pixel = s.ppu_buffer;
          s.ppu_line_y = 0;
          SetPPUMode(2);
          set_stat_mode(0);
        }
        s.ppu_line_start_tick = s.tick + 1;
      }
      break;

    case 2:
      if (line_tick == 1 && s.ppu_line_y == 0) {
        set_stat_mode(2);
      }
      StepPPU_Mode2(mode_tick);
      break;

    case 3: StepPPU_Mode3(mode_tick); break;
  }
}

void GB::StepPPU_Mode2(Tick mode_tick) {
  // TODO: sprite stuff
  assert(mode_tick < 160);
  if (mode_tick == 159) {
    SetPPUMode(3);
    s.ppu_window = false;
    s.ppu_line_x = 0;
    s.ppu_stall = 14 + (s.io[SCX] & 7);
  }
}

void GB::StepPPU_Mode3(Tick mode_tick) {
  assert(mode_tick < 362);
  switch (mode_tick) {
    case 0: {
      if (s.ppu_window) {
        s.ppu_draw_y = s.ppu_line_y - s.io[WY];
        s.ppu_map_addr = (s.io[LCDC] & 0x40) << 4;
      } else {
        s.ppu_draw_y = s.ppu_line_y + s.io[SCY];
        s.ppu_map_addr = ((s.io[LCDC] & 8) << 7) | ((s.io[SCX] & 0xf8) >> 3);
      }
      s.ppu_map_addr |= 0x1800 | ((s.ppu_draw_y & 0xf8) << 2);
      s.ppu_map = s.vram[s.ppu_map_addr];
      break;
    }

    case 12:
      if (!s.ppu_window) { goto no_increment; }
      // Fallthrough.

    case 28: case 44: case 60: case 76: case 92: case 108:
    case 124: case 140: case 156: case 172: case 188: case 204: case 220:
    case 236: case 252: case 268: case 284: case 300: case 316: case 332:
    case 348:
      s.ppu_map_addr = (s.ppu_map_addr & ~31) | ((s.ppu_map_addr + 1) & 31);

    no_increment:
      // Next map addr.
      s.ppu_map = s.vram[s.ppu_map_addr];
      s.ppu_tile[0] = s.ppu_next_tile[0];
      s.ppu_tile[1] = s.ppu_next_tile[1];
      break;

    case 4: case 16: case 32: case 48: case 64: case 80: case 96: case 112:
    case 128: case 144: case 160: case 176: case 192: case 208: case 224:
    case 240: case 256: case 272: case 288: case 304: case 320: case 336:
    case 352:
      // Fetch plane 0.
      s.ppu_tile_addr = (s.ppu_draw_y & 0x7) << 1;
      if (s.io[LCDC] & 0x10) {
        s.ppu_tile_addr |= (s.ppu_map << 4);
      } else {
        s.ppu_tile_addr |= (0x1000 + ((s16)(s8)s.ppu_map << 4));
      }
      s.ppu_next_tile[0] = s.vram[s.ppu_tile_addr++];
      break;

    case 8: case 20: case 36: case 52: case 68: case 84: case 100: case 116:
    case 132: case 148: case 164: case 180: case 196: case 212: case 228:
    case 244: case 260: case 276: case 292: case 308: case 324: case 340:
    case 356:
      // Fetch plane 1.
      s.ppu_next_tile[1] = s.vram[s.ppu_tile_addr];
      break;

    case 24: case 40: case 56: case 72: case 88: case 104: case 120:
    case 136: case 152: case 168: case 184: case 200: case 216: case 232:
    case 248: case 264: case 280: case 296: case 312: case 328: case 344:
    case 360:
      // Sprite window.
      break;
  }

  if (falling(s.tick)) {
    if (s.ppu_stall == 0) {
      u8 pal_index = ((s.ppu_tile[1] >> 6) & 2) | (s.ppu_tile[0] >> 7);
      *s.ppu_pixel++ = (s.io[BGP] >> (pal_index * 2)) & 3;
      if (++s.ppu_line_x == 160) {
        SetPPUMode(0);
      }
    } else {
      --s.ppu_stall;
    }

    s.ppu_tile[0] <<= 1;
    s.ppu_tile[1] <<= 1;

    if (!s.ppu_window && (s.io[LCDC] & 0x20) && s.ppu_line_y >= s.io[WY] &&
        mode_tick == (s.io[WX] + 6) * 2) {
      s.ppu_stall = 6;
      s.ppu_mode_start_tick = s.tick + 2;
      s.ppu_window = true;
    }
  }
}

void GB::SetPPUMode(u8 mode) {
  u8& stat = s.io[STAT];
  u8& if_ = s.io[IF];
  set_stat_mode(mode);
  s.ppu_mode = mode;
  s.ppu_mode_start_tick = s.tick + 1;
  DPRINT(MODE, "(%lu) mode: %d\n", s.ppu_mode_start_tick - s.ppu_lcd_on_tick,
         mode);
  u8 old = if_;
  switch (mode) {
    case 0: if (stat & 0x08) { if_ |= 2; } break;
    case 1: if (stat & 0x10) { if_ |= 2; } if_ |= 1; break;
    case 2: if (stat & 0x20) { if_ |= 2; } break;
  }
  if (if_ != old) {
    DPRINT(INTERRUPTS, "trigger IF %d mode: %d\n", (if_ ^ old) & if_, mode);
  }
}

void GB::CheckLyLyc() {
  s.io[STAT] = (s.io[STAT] & ~4) | (!!(s.io[LY] == s.io[LYC]) << 2);
  if ((s.io[STAT] & 0x44) == 0x44) {
    s.io[IF] |= 2;
    DPRINT(INTERRUPTS, "trigger STAT IF 2 ly=lyc=%d\n", s.io[LY]);
  }
}


static u8 s_opcode_bytes[] = {
    /*       0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f */
    /* 00 */ 1, 3, 1, 1, 1, 1, 2, 1, 3, 1, 1, 1, 1, 1, 2, 1,
    /* 10 */ 1, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
    /* 20 */ 2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
    /* 30 */ 2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
    /* 40 */ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    /* 50 */ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    /* 60 */ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    /* 70 */ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    /* 80 */ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    /* 90 */ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    /* a0 */ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    /* b0 */ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    /* c0 */ 1, 1, 3, 3, 3, 1, 2, 1, 1, 1, 3, 2, 3, 3, 2, 1,
    /* d0 */ 1, 1, 3, 0, 3, 1, 2, 1, 1, 1, 3, 0, 3, 0, 2, 1,
    /* e0 */ 2, 1, 1, 0, 0, 1, 2, 1, 2, 1, 3, 0, 0, 0, 2, 1,
    /* f0 */ 2, 1, 1, 1, 0, 1, 2, 1, 2, 1, 3, 1, 0, 0, 2, 1,
};

static const char* s_opcode_mnemonic[256] = {
  "nop", "ld bc,%hu", "ld [bc],a", "inc bc", "inc b", "dec b", "ld b,%hhu",
  "rlca", "ld [$%04x],sp", "add hl,bc", "ld a,[bc]", "dec bc", "inc c", "dec c",
  "ld c,%hhu", "rrca", "stop", "ld de,%hu", "ld [de],a", "inc de", "inc d",
  "dec d", "ld d,%hhu", "rla", "jr %+hhd", "add hl,de", "ld a,[de]", "dec de",
  "inc e", "dec e", "ld e,%hhu", "rra", "jr nz,%+hhd", "ld hl,%hu",
  "ld [hl+],a", "inc hl", "inc h", "dec h", "ld h,%hhu", "daa", "jr z,%+hhd",
  "add hl,hl", "ld a,[hl+]", "dec hl", "inc l", "dec l", "ld l,%hhu", "cpl",
  "jr nc,%+hhd", "ld sp,%hu", "ld [hl-],a", "inc sp", "inc [hl]", "dec [hl]",
  "ld [hl],%hhu", "scf", "jr c,%+hhd", "add hl,sp", "ld a,[hl-]", "dec sp",
  "inc a", "dec a", "ld a,%hhu", "ccf", "ld b,b", "ld b,c", "ld b,d", "ld b,e",
  "ld b,h", "ld b,l", "ld b,[hl]", "ld b,a", "ld c,b", "ld c,c", "ld c,d",
  "ld c,e", "ld c,h", "ld c,l", "ld c,[hl]", "ld c,a", "ld d,b", "ld d,c",
  "ld d,d", "ld d,e", "ld d,h", "ld d,l", "ld d,[hl]", "ld d,a", "ld e,b",
  "ld e,c", "ld e,d", "ld e,e", "ld e,h", "ld e,l", "ld e,[hl]", "ld e,a",
  "ld h,b", "ld h,c", "ld h,d", "ld h,e", "ld h,h", "ld h,l", "ld h,[hl]",
  "ld h,a", "ld l,b", "ld l,c", "ld l,d", "ld l,e", "ld l,h", "ld l,l",
  "ld l,[hl]", "ld l,a", "ld [hl],b", "ld [hl],c", "ld [hl],d", "ld [hl],e",
  "ld [hl],h", "ld [hl],l", "halt", "ld [hl],a", "ld a,b", "ld a,c", "ld a,d",
  "ld a,e", "ld a,h", "ld a,l", "ld a,[hl]", "ld a,a", "add a,b", "add a,c",
  "add a,d", "add a,e", "add a,h", "add a,l", "add a,[hl]", "add a,a",
  "adc a,b", "adc a,c", "adc a,d", "adc a,e", "adc a,h", "adc a,l",
  "adc a,[hl]", "adc a,a", "sub a,b", "sub a,c", "sub a,d", "sub a,e",
  "sub a,h", "sub a,l", "sub a,[hl]", "sub a,a", "sbc a,b", "sbc a,c",
  "sbc a,d", "sbc a,e", "sbc a,h", "sbc a,l", "sbc a,[hl]", "sbc a,a",
  "and a,b", "and a,c", "and a,d", "and a,e", "and a,h", "and a,l",
  "and a,[hl]", "and a,a", "xor a,b", "xor a,c", "xor a,d", "xor a,e",
  "xor a,h", "xor a,l", "xor a,[hl]", "xor a,a", "or a,b", "or a,c", "or a,d",
  "or a,e", "or a,h", "or a,l", "or a,[hl]", "or a,a", "cp a,b", "cp a,c",
  "cp a,d", "cp a,e", "cp a,h", "cp a,l", "cp a,[hl]", "cp a,a", "ret nz",
  "pop bc", "jp nz,$%04hx", "jp $%04hx", "call nz,$%04hx", "push bc",
  "add a,%hhu", "rst $00", "ret z", "ret", "jp z,$%04hx", NULL, "call z,$%04hx",
  "call $%04hx", "adc a,%hhu", "rst $08", "ret nc", "pop de", "jp nc,$%04hx",
  NULL, "call nc,$%04hx", "push de", "sub a,%hhu", "rst $10", "ret c", "reti",
  "jp c,$%04hx", NULL, "call c,$%04hx", NULL, "sbc a,%hhu", "rst $18",
  "ldh [$ff%02hhx],a", "pop hl", "ld [$ff00+c],a", NULL, NULL, "push hl",
  "and a,%hhu", "rst $20", "add sp,%hhd", "jp hl", "ld [$%04hx],a", NULL, NULL,
  NULL, "xor a,%hhu", "rst $28", "ldh a,[$ff%02hhx]", "pop af",
  "ld a,[$ff00+c]", "di", NULL, "push af", "or a,%hhu", "rst $30",
  "ld hl,sp%+hhd", "ld sp,hl", "ld a,[$%04hx]", "ei", NULL, NULL, "cp a,%hhu",
  "rst $38",
};

static const char* s_cb_opcode_mnemonic[256] = {
    "rlc b",      "rlc c",   "rlc d",      "rlc e",   "rlc h",      "rlc l",
    "rlc [hl]",   "rlc a",   "rrc b",      "rrc c",   "rrc d",      "rrc e",
    "rrc h",      "rrc l",   "rrc [hl]",   "rrc a",   "rl b",       "rl c",
    "rl d",       "rl e",    "rl h",       "rl l",    "rl [hl]",    "rl a",
    "rr b",       "rr c",    "rr d",       "rr e",    "rr h",       "rr l",
    "rr [hl]",    "rr a",    "sla b",      "sla c",   "sla d",      "sla e",
    "sla h",      "sla l",   "sla [hl]",   "sla a",   "sra b",      "sra c",
    "sra d",      "sra e",   "sra h",      "sra l",   "sra [hl]",   "sra a",
    "swap b",     "swap c",  "swap d",     "swap e",  "swap h",     "swap l",
    "swap [hl]",  "swap a",  "srl b",      "srl c",   "srl d",      "srl e",
    "srl h",      "srl l",   "srl [hl]",   "srl a",   "bit 0,b",    "bit 0,c",
    "bit 0,d",    "bit 0,e", "bit 0,h",    "bit 0,l", "bit 0,[hl]", "bit 0,a",
    "bit 1,b",    "bit 1,c", "bit 1,d",    "bit 1,e", "bit 1,h",    "bit 1,l",
    "bit 1,[hl]", "bit 1,a", "bit 2,b",    "bit 2,c", "bit 2,d",    "bit 2,e",
    "bit 2,h",    "bit 2,l", "bit 2,[hl]", "bit 2,a", "bit 3,b",    "bit 3,c",
    "bit 3,d",    "bit 3,e", "bit 3,h",    "bit 3,l", "bit 3,[hl]", "bit 3,a",
    "bit 4,b",    "bit 4,c", "bit 4,d",    "bit 4,e", "bit 4,h",    "bit 4,l",
    "bit 4,[hl]", "bit 4,a", "bit 5,b",    "bit 5,c", "bit 5,d",    "bit 5,e",
    "bit 5,h",    "bit 5,l", "bit 5,[hl]", "bit 5,a", "bit 6,b",    "bit 6,c",
    "bit 6,d",    "bit 6,e", "bit 6,h",    "bit 6,l", "bit 6,[hl]", "bit 6,a",
    "bit 7,b",    "bit 7,c", "bit 7,d",    "bit 7,e", "bit 7,h",    "bit 7,l",
    "bit 7,[hl]", "bit 7,a", "res 0,b",    "res 0,c", "res 0,d",    "res 0,e",
    "res 0,h",    "res 0,l", "res 0,[hl]", "res 0,a", "res 1,b",    "res 1,c",
    "res 1,d",    "res 1,e", "res 1,h",    "res 1,l", "res 1,[hl]", "res 1,a",
    "res 2,b",    "res 2,c", "res 2,d",    "res 2,e", "res 2,h",    "res 2,l",
    "res 2,[hl]", "res 2,a", "res 3,b",    "res 3,c", "res 3,d",    "res 3,e",
    "res 3,h",    "res 3,l", "res 3,[hl]", "res 3,a", "res 4,b",    "res 4,c",
    "res 4,d",    "res 4,e", "res 4,h",    "res 4,l", "res 4,[hl]", "res 4,a",
    "res 5,b",    "res 5,c", "res 5,d",    "res 5,e", "res 5,h",    "res 5,l",
    "res 5,[hl]", "res 5,a", "res 6,b",    "res 6,c", "res 6,d",    "res 6,e",
    "res 6,h",    "res 6,l", "res 6,[hl]", "res 6,a", "res 7,b",    "res 7,c",
    "res 7,d",    "res 7,e", "res 7,h",    "res 7,l", "res 7,[hl]", "res 7,a",
    "set 0,b",    "set 0,c", "set 0,d",    "set 0,e", "set 0,h",    "set 0,l",
    "set 0,[hl]", "set 0,a", "set 1,b",    "set 1,c", "set 1,d",    "set 1,e",
    "set 1,h",    "set 1,l", "set 1,[hl]", "set 1,a", "set 2,b",    "set 2,c",
    "set 2,d",    "set 2,e", "set 2,h",    "set 2,l", "set 2,[hl]", "set 2,a",
    "set 3,b",    "set 3,c", "set 3,d",    "set 3,e", "set 3,h",    "set 3,l",
    "set 3,[hl]", "set 3,a", "set 4,b",    "set 4,c", "set 4,d",    "set 4,e",
    "set 4,h",    "set 4,l", "set 4,[hl]", "set 4,a", "set 5,b",    "set 5,c",
    "set 5,d",    "set 5,e", "set 5,h",    "set 5,l", "set 5,[hl]", "set 5,a",
    "set 6,b",    "set 6,c", "set 6,d",    "set 6,e", "set 6,h",    "set 6,l",
    "set 6,[hl]", "set 6,a", "set 7,b",    "set 7,c", "set 7,d",    "set 7,e",
    "set 7,h",    "set 7,l", "set 7,[hl]", "set 7,a",
};

void GB::Print(const char* fmt, ...) const {
  va_list args;
  va_start(args, fmt);
  printf("%lu (%.1f): ", s.tick, clocks());
  vprintf(fmt, args);
  va_end(args);
}

static void sprint_hex(char* buffer, u8 val) {
  const char hex_digits[] = "0123456789abcdef";
  buffer[0] = hex_digits[(val >> 4) & 0xf];
  buffer[1] = hex_digits[val & 0xf];
}

int GB::Disassemble(u16 addr, char* buffer, size_t size) {
  char temp[64];
  char bytes[][3] = {"  ", "  "};
  const char* mnemonic = "*INVALID*";

  u8 opcode = Read(addr);
  u8 num_bytes = s_opcode_bytes[opcode];
  switch (num_bytes) {
    case 0: break;
    case 1: mnemonic = s_opcode_mnemonic[opcode]; break;
    case 2: {
      u8 byte = Read(addr + 1);
      sprint_hex(bytes[0], byte);
      if (opcode == 0xcb) {
        mnemonic = s_cb_opcode_mnemonic[byte];
      } else {
        snprintf(temp, sizeof(temp), s_opcode_mnemonic[opcode], byte);
        mnemonic = temp;
      }
      break;
    }
    case 3: {
      u8 byte1 = Read(addr + 1);
      u8 byte2 = Read(addr + 2);
      sprint_hex(bytes[0], byte1);
      sprint_hex(bytes[1], byte2);
      snprintf(temp, sizeof(temp), s_opcode_mnemonic[opcode],
               (byte2 << 8) | byte1);
      mnemonic = temp;
      break;
    }
    default: assert(!"invalid opcode byte length.\n"); break;
  }

  char bank[3] = "??";
  if ((addr >> 12) < 4) {
    sprint_hex(bank, (s.rom0p - rom.data()) >> 14);
  } else if ((addr >> 12) < 8) {
    sprint_hex(bank, (s.rom1p - rom.data()) >> 14);
  }

  snprintf(buffer, size, "[%s]%#06x: %02x %s %s  %-15s", bank, addr, opcode,
           bytes[0], bytes[1], mnemonic);
  return num_bytes ? num_bytes : 1;
}
void GB::PrintInstruction(u16 addr) {
  char temp[64];
  Disassemble(addr, temp, sizeof(temp));
  printf("%s", temp);
}

void GB::Trace() {
  if (s.op_tick == 0 && (s.tick & 1) == 0) {
    printf("a:%02x f:%c%c%c%c bc:%04x de:%04x hl:%04x sp:%04x pc:%04x", s.a,
           (s.f & FZ) ? 'z' : '-', (s.f & FN) ? 'n' : '-',
           (s.f & FH) ? 'h' : '-', (s.f & FC) ? 'c' : '-', s.bc, s.de, s.hl,
           s.sp, s.pc);
    printf(" (cy: %.f)", clocks());
    printf(" ppu:%c%u", s.io[LCDC] & 0x80 ? '+' : '-', s.io[STAT] & 3);
    printf(" LY:%u", s.io[LY]);
    printf(" |");
    PrintInstruction(s.pc);
    printf("\n");
  }
}


Buffer ReadFile(const char* filename) {
  std::ifstream file(filename, std::ios::binary | std::ios::in | std::ios::ate);
  if (!file) {
    throw Error(std::string("Failed to open file \"") + filename + "\".");
  }

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  Buffer data(size);
  file.read(reinterpret_cast<char*>(data.data()), data.size());

  if (file.bad()) {
    throw Error("Failed to read file.");
  }

  return data;
}

void WriteFramePPM(const GB& gb, const char* filename) {
  std::ofstream file(filename, std::ios::out);
  if (!file) {
    throw Error(std::string("Failed to open file \"") + filename + "\".");
  }
  u32 colors[] = {0xffffffffu, 0xffaaaaaau, 0xff555555u, 0xff000000u};
  file << "P3\n160 144\n255\n";
  u8 x, y;
  const u8* data = gb.s.ppu_buffer;
  for (y = 0; y < 144; ++y) {
    for (x = 0; x < 160; ++x) {
      u32 pixel = colors[*data++];
      int b = (pixel >> 16) & 0xff;
      int g = (pixel >> 8) & 0xff;
      int r = (pixel >> 0) & 0xff;
      file << std::setw(3) << r << ' ' << std::setw(3) << g << ' '
           << std::setw(3) << b << ' ';
    }
    file << '\n';
  }
  if (file.bad()) {
    throw Error("Failed to write file.");
  }
}

static const char* s_filename = nullptr;
static const char* s_ppm_filename = nullptr;
static bool s_trace = false;
static int s_frames = 60;

void ParseArguments(int argc, char** argv) {
  --argc;
  ++argv;
  for (; argc; --argc, ++argv) {
    const char* arg = *argv;
    if (arg[0] == '-') {
      switch (arg[1]) {
        case 'f':
          if (--argc == 0) {
            throw Error("frame count required for -f");
          }
          try {
            s_frames = std::stoi(*++argv);
          } catch (const std::invalid_argument&) {
            throw Error("invalid frame count");
          }
          break;

        case 't':
          s_trace = true;
          break;

        case 'o':
          if (--argc == 0) {
            throw Error("filename require for -o");
          }
          s_ppm_filename = *++argv;
          break;
      }
    } else if (s_filename) {
      throw Error("multiple filenames");
    } else {
      s_filename = arg;
    }
  }

  if (!s_filename) {
    throw Error("No rom file given.");
  }
}

f64 GetTime() {
  timeval tp;
  gettimeofday(&tp, nullptr);
  return (f64)tp.tv_sec + (f64)tp.tv_usec / 1000000.0;
}

int main(int argc, char** argv) {
  try {
    ParseArguments(argc, argv);

    GB gb(ReadFile(s_filename), Variant::Guess);

    Clock run_clocks = s_frames * 70224u;
    f64 start_time = GetTime();
    if (s_trace) {
      for (Clock i = 0; i < run_clocks; ++i) {
        gb.Trace();
        gb.Step();
      }
    } else {
      for (Clock i = 0; i < run_clocks; ++i) {
        gb.Step();
      }
    }
    f64 host_time = GetTime() - start_time;
    f64 gb_time = (f64)run_clocks / CpuClocksPerSec;
    printf("time: gb=%.1fs host=%.1fs (%.1fx)\n", gb_time, host_time,
           gb_time / host_time);

    if (s_ppm_filename) {
      WriteFramePPM(gb, s_ppm_filename);
    }

    return 0;
  } catch (const Error& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
