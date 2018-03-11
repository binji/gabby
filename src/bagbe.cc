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
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using Tick = uint64_t;
using Error = std::runtime_error;
using Buffer = std::vector<u8>;

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

enum class MBC { None, _1, _2, _3, _4, _5, MMM01, TAMA5, HUC3, HUC1 };
enum class CGB { None, Supported, Required };
enum class ROMSize { _32k, _64k, _128k, _256k, _512k, _1m, _2m, _4m, _8m };
enum class SRAMSize { None, _2k, _8k, _32k, _128k, _64k };
enum class Variant { Guess, DMG0, DMGABCX, MGB, SGB, CGB };

struct Cart {
  Cart(const Buffer&, Variant);

  Variant variant;
  MBC mbc;
  CGB cgb;
  ROMSize rom_size;
  SRAMSize sram_size;
};

struct GB;
struct State {
  explicit State(GB&);

  Tick tick, op_tick, ppu_line_tick;
  union { struct { u8 f, a; }; u16 af; };
  union { struct { u8 c, b; }; u16 bc; };
  union { struct { u8 e, d; }; u16 de; };
  union { struct { u8 l, h; }; u16 hl; };
  union { struct { u8 z, w; }; u16 wz; };
  u16 sp, pc;
  u8 *rom0p, *rom1p, *vramp, *wram0p, *wram1p, *sramp;
  u8 vram[0x4000]{};  // 0x8000-0x9fff, 2 banks
  u8 sram[0x8000]{};  // 0xa000-0xbfff, 4 banks
  u8 wram[0x8000]{};  // 0xc000-0xdfff, 4 banks
  u8 oam[0x100]{};    // 0xfe00-0xfe9f
  u8 io[0x100]{};     // 0xff00-0xffff
  u8 op, cb_op;
  bool ime, ime_enable, dispatch;
};

struct GB {
  explicit GB(Buffer&&, Variant);
  void StepCPU();
  void DispatchInterrupt();
  void StepPPU();

  int Disassemble(u16 addr, char* buffer, size_t size);
  void PrintInstruction(u16 addr);
  void Trace();

  u8 ReadU8(u16 addr);
  void WriteU8(u16 addr, u8 val);
  void WriteU8IO(u8 addr, u8 val);

  static u8 zflag(u8);
  static u8 hflag(int res, int x, int y);
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
  void call_f_nn(u8 mask, u8 val);
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
  void jp_f_nn(u8 mask, u8 val);
  void jp_hl();
  void jp_nn();
  void jr_f_n(u8 mask, u8 val);
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
  void ret_f(u8 mask, u8 val);
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
    case 21: case 22: case 23: mbc = MBC::_4; break;
    case 25: case 26: case 27: case 28: case 29: case 30: mbc = MBC::_5; break;
    case 253: mbc = MBC::TAMA5; break;
    case 254: mbc = MBC::HUC3; break;
    case 255: mbc = MBC::HUC1; break;
    default: throw Error("Invalid cart type");
  }

  ThrowUnless(rom[0x148] <= 8, "Invalid ROM size.");
  rom_size = static_cast<ROMSize>(rom[0x148]);

  ThrowUnless(rom[0x149] <= 5, "Invalid SRAM size.");
  sram_size = static_cast<SRAMSize>(rom[0x149]);
}

State::State(GB& gb) {
  tick = op_tick = ppu_line_tick = 0;
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
  ime = ime_enable = dispatch = false;

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

void GB::StepCPU() {
  switch (++s.op_tick) {
    case 1:
      s.ime = s.ime_enable ? true : s.ime;
      s.ime_enable = false;
      break;
    case 2:
      s.dispatch = s.ime && !!(s.io[IF] & s.io[IE] & 0x1f);
      break;
    case 3:
      s.op = ReadU8(s.pc++);
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
        case 0x20: jr_f_n(0x80, 0); break;
        case 0x21: ld_rr_nn(s.hl); break;
        case 0x22: ld_mr_r(s.hl, s.a, 1); break;
        case 0x23: inc_rr(s.hl); break;
        case 0x24: inc_r(s.h); break;
        case 0x25: dec_r(s.h); break;
        case 0x26: ld_r_n(s.h); break;
        case 0x27: daa(); break;
        case 0x28: jr_f_n(0x80, 0x80); break;
        case 0x29: add_hl_rr(s.hl); break;
        case 0x2a: ld_r_mr(s.a, s.hl, 1); break;
        case 0x2b: dec_rr(s.hl); break;
        case 0x2c: inc_r(s.l); break;
        case 0x2d: dec_r(s.l); break;
        case 0x2e: ld_r_n(s.l); break;
        case 0x2f: cpl(); break;
        case 0x30: jr_f_n(0x10, 0); break;
        case 0x31: ld_rr_nn(s.sp); break;
        case 0x32: ld_mr_r(s.hl, s.a, -1); break;
        case 0x33: inc_rr(s.sp); break;
        case 0x34: inc_mhl(); break;
        case 0x35: dec_mhl(); break;
        case 0x36: ld_mhl_n(); break;
        case 0x37: scf(); break;
        case 0x38: jr_f_n(0x10, 0x10); break;
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
        case 0xc0: ret_f(0x80, 0); break;
        case 0xc1: pop_rr(s.bc); break;
        case 0xc2: jp_f_nn(0x80, 0); break;
        case 0xc3: jp_nn(); break;
        case 0xc4: call_f_nn(0x80, 0); break;
        case 0xc5: push_rr(s.bc); break;
        case 0xc6: add_n(); break;
        case 0xc7: rst(0x00); break;
        case 0xc8: ret_f(0x80, 0x80); break;
        case 0xc9: ret(); break;
        case 0xca: jp_f_nn(0x80, 0x80); break;
        case 0xcb: cb(); break;
        case 0xcc: call_f_nn(0x80, 0x80); break;
        case 0xcd: call_nn(); break;
        case 0xce: adc_n(); break;
        case 0xcf: rst(0x08); break;
        case 0xd0: ret_f(0x10, 0); break;
        case 0xd1: pop_rr(s.de); break;
        case 0xd2: jp_f_nn(0x10, 0); break;
        case 0xd4: call_f_nn(0x10, 0); break;
        case 0xd5: push_rr(s.de); break;
        case 0xd6: sub_n(); break;
        case 0xd7: rst(0x10); break;
        case 0xd8: ret_f(0x10, 0x10); break;
        case 0xd9: reti(); break;
        case 0xda: jp_f_nn(0x10, 0x10); break;
        case 0xdc: call_f_nn(0x10, 0x10); break;
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
  ++s.tick;
}

void GB::cb() {
  switch (s.op_tick) {
    case 4: case 5: case 6:
      break;
    case 7:
      s.cb_op = ReadU8(s.pc++);
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
    case 7: WriteU8(--s.sp, s.pc >> 8); break;
    case 8: {
      u8& if_ = s.io[IF];
      u8 intr = if_ & s.io[IE] & 0x1f;
      s.wz = 0;
      for (int i = 0; i < 5; ++i) {
        if (if_ & (1 << i)) {
          if_ &= ~(1 << i);
          s.wz = 0x40 + (i << 3);
          break;
        }
      }
      break;
    }
    case 11: WriteU8(--s.sp, s.pc); s.pc = s.wz; break;
    case 20: s.op_tick = 0; break;
  }
}

u8 GB::ReadU8(u16 addr) {
  switch (addr >> 12) {
    case 0: case 1: case 2: case 3: return s.rom0p[addr & 0x3fff];
    case 4: case 5: case 6: case 7: return s.rom1p[addr & 0x3fff];
    case 8: case 9: return s.vramp[addr & 0x1fff];
    case 10: case 11: return s.sramp[addr & 0x1fff];
    case 12: case 14: return s.wram0p[addr & 0xfff];
    case 13: return s.wram1p[addr & 0xfff];
    case 15:
      switch ((addr >> 8) & 0xf) {
        default: return s.wram1p[addr & 0xfff];
        case 14: return s.oam[addr & 0xff];
        case 15: return s.io[addr & 0xff];
      }
  }
  return 0xff;
}

void GB::WriteU8(u16 addr, u8 val) {
  switch (addr >> 12) {
    case 8: case 9: s.vramp[addr & 0x1fff] = val; break;
    case 10: case 11: s.sramp[addr & 0x1fff] = val; break;
    case 12: case 14: s.wram0p[addr & 0xfff] = val; break;
    case 13: s.wram1p[addr & 0xfff] = val; break;
    case 15:
      switch ((addr >> 8) & 0xf) {
        default: s.wram1p[addr & 0xfff] = val; break;
        case 14: s.oam[addr & 0xff] = val; break;
        case 15: WriteU8IO(addr & 0xff, val); break;
      }
  }
}

void GB::WriteU8IO(u8 addr, u8 val) {
  static const u8 dmg_io_mask[256] = {
      0x30, 0xff, 0x81, 0,    0,    0xff, 0xff, 0x07, 0,    0,    0,    0,
      0,    0,    0,    0xff, 0x7f, 0xff, 0xff, 0xff, 0xc0, 0,    0xff, 0xff,
      0xff, 0xc0, 0x80, 0xff, 0x60, 0xff, 0xc0, 0,    0xff, 0xff, 0xff, 0xc0,
      0xff, 0xff, 0x8f, 0,    0,    0,    0,    0,    0,    0,    0,    0,
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
    case LCDC: s.io[LY] = 0; s.ppu_line_tick = 0; break;
  }
}

// static
u8 GB::zflag(u8 x) { return x ? 0 : 0x80; }

// static
u8 GB::hflag(int res, int x, int y) { return ((res ^ x ^ y) << 1) & 0x20; }

bool GB::f_is(u8 mask, u8 val) {
  if ((s.f & mask) == val) {
    return true;
  }
  s.op_tick = 0;
  return false;
}

void GB::add(u8 x, u8 c) {
  int r = s.a + x + c;
  s.f = zflag(r) | hflag(r, s.a, x) | ((r >> 4) & 0x10);
  s.a = r;
}

void GB::adc_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 8: add(s.z, (s.f >> 4) & 1); s.op_tick = 0; break;
  }
}

void GB::adc_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: add(s.z, (s.f >> 4) & 1); s.op_tick = 0; break;
  }
}

void GB::adc_r(u8 r) {
  switch (s.op_tick) {
    case 4: add(r, (s.f >> 4) & 1); s.op_tick = 0; break;
  }
}

void GB::add_hl_rr(u16 rr) {
  switch (s.op_tick) {
    case 8: {
      int res = s.hl + rr;
      s.f = (s.f & 0x80) | (((s.hl ^ rr ^ res) >> 7) & 0x20) |
            ((res >> 12) & 0x10);
      s.hl = res;
      s.op_tick = 0;
      break;
    }
  }
}

void GB::add_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 8: add(s.z); s.op_tick = 0; break;
  }
}

void GB::add_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: add(s.z); s.op_tick = 0; break;
  }
}

void GB::add_r(u8 r) {
  switch (s.op_tick) {
    case 4: add(r); s.op_tick = 0; break;
  }
}

u16 GB::add_sp(u8 x) {
  int res = (u8)s.sp + x;
  s.f = hflag(res, (u8)s.sp, x) | ((res >> 4) & 0x10);
  return s.sp + (s8)x;
}

void GB::add_sp_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: s.sp = add_sp(s.z); break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::and_(u8 x) {
  s.a &= x;
  s.f = zflag(s.a) | 0x20;
}

void GB::and_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 8: and_(s.z); s.op_tick = 0; break;
  }
}

void GB::and_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: and_(s.z); s.op_tick = 0; break;
  }
}

void GB::and_r(u8 r) {
  switch (s.op_tick) {
    case 4: and_(r); s.op_tick = 0; break;
  }
}

void GB::bit(int n, u8 r) { s.f = (s.f & 0x10) | zflag(r & (1 << n)) | 0x20; }

void GB::bit_r(int n, u8 r) {
  switch (s.op_tick) {
    case 8: bit(n, r); s.op_tick = 0; break;
  }
}

void GB::bit_mr(int n, u16 mr) {
  switch (s.op_tick) {
    case 11: s.z = ReadU8(mr); break;
    case 12: bit(n, s.z); s.op_tick = 0; break;
  }
}

void GB::call_f_nn(u8 mask, u8 val) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: s.w = ReadU8(s.pc++); break;
    case 12: f_is(mask, val); break;
    case 19: WriteU8(--s.sp, s.pc >> 8); break;
    case 23: WriteU8(--s.sp, s.pc); s.pc = s.wz; break;
    case 24: s.op_tick = 0; break;
  }
}

void GB::call_nn() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: s.w = ReadU8(s.pc++); break;
    case 19: WriteU8(--s.sp, s.pc >> 8); break;
    case 23: WriteU8(--s.sp, s.pc); s.pc = s.wz; break;
    case 24: s.op_tick = 0; break;
  }
}

void GB::ccf() {
  switch (s.op_tick) {
    case 4: s.f = (s.f & 0x80) | ((s.f ^ 0x10) & 0x10); s.op_tick = 0; break;
  }
}

void GB::cpl() {
  switch (s.op_tick) {
    case 4: s.a = ~s.a; s.f = (s.f & 0x90) | 0x60; s.op_tick = 0; break;
  }
}

void GB::cp_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 8: sub(s.z); s.op_tick = 0; break;
  }
}

void GB::cp_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: sub(s.z); s.op_tick = 0; break;
  }
}

void GB::cp_r(u8 r) {
  switch (s.op_tick) {
    case 4: sub(r); s.op_tick = 0; break;
  }
}

void GB::daa() {
  switch (s.op_tick) {
    case 4: {
      u8 r = 0;
      if ((s.f & 0x20) || (!(s.f & 0x40) && (s.a & 0xf) > 9)) {
        r = 6;
      }
      if ((s.f & 0x10) || (!(s.f & 0x40) && s.a > 0x99)) {
        r |= 0x60;
        s.f |= 0x10;
      }
      s.a += (s.f & 0x40) ? -r : r;
      s.f = (s.f & 0x50) | (s.a ? 0 : 0x80);
      s.op_tick = 0;
      break;
    }
  }
}

u8 GB::dec(u8 r) {
  int res = r - 1;
  s.f = (s.f & 0x10) | zflag(res) | 0x40 | hflag(res, r, 0);
  return res;
}

void GB::dec_mhl() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.hl); break;
    case 11: WriteU8(s.hl, dec(s.z)); break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::dec_rr(u16& rr) {
  switch (s.op_tick) {
    case 4: rr--; break;
    case 8: s.op_tick = 0; break;
  }
}

void GB::dec_r(u8& r) {
  switch (s.op_tick) {
    case 4: r = dec(r); s.op_tick = 0; break;
  }
}

void GB::di() {
  switch (s.op_tick) {
    case 4: s.ime_enable = s.ime = false; s.op_tick = 0; break;
  }
}

void GB::ei() {
  switch (s.op_tick) {
    case 4: s.ime_enable = true; s.op_tick = 0; break;
  }
}

void GB::halt() {
  if (((s.op_tick & 3) == 0) && (s.io[IF] & s.io[IE] & 0x1f)) {
    s.op = ReadU8(s.pc);
    if (s.ime) {
      s.dispatch = true;
    } else if (s.op_tick > 4) {
      s.pc++;
    }
    s.op_tick = 4;
  }
}

u8 GB::inc(u8 r) {
  int res = r + 1;
  s.f = (s.f & 0x10) | zflag(res) | hflag(res, r, 0);
  return res;
}

void GB::inc_mhl() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.hl); break;
    case 11: WriteU8(s.hl, inc(s.z)); break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::inc_rr(u16& rr) {
  switch (s.op_tick) {
    case 4: rr++; break;
    case 8: s.op_tick = 0; break;
  }
}

void GB::inc_r(u8& r) {
  switch (s.op_tick) {
    case 4: r = inc(r); s.op_tick = 0; break;
  }
}

void GB::jp_f_nn(u8 mask, u8 val) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: s.w = ReadU8(s.pc++); break;
    case 12: if (f_is(mask, val)) { s.pc = s.wz; } break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::jp_hl() {
  switch (s.op_tick) {
    case 4: s.pc = s.hl; s.op_tick = 0; break;
  }
}

void GB::jp_nn() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: s.w = ReadU8(s.pc++); break;
    case 12: s.pc = s.wz; break;
    case 16: s.pc = s.wz; s.op_tick = 0; break;
  }
}

void GB::jr_f_n(u8 mask, u8 val) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: if (f_is(mask, val)) { s.pc += (s8)s.z; } break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::jr_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: s.pc += (s8)s.z; break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::ld_a_mff00_c() {
  switch (s.op_tick) {
    case 7: s.a = ReadU8(0xff00 + s.c); break;
    case 8: s.op_tick = 0; break;
  }
}

void GB::ld_a_mff00_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: s.a = ReadU8(0xff00 + s.z); break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::ld_a_mnn() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: s.w = ReadU8(s.pc++); break;
    case 15: s.a = ReadU8(s.wz); break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::ld_a_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.a = ReadU8(mr); break;
    case 8: s.op_tick = 0; break;
  }
}

void GB::ld_hl_sp_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: s.hl = add_sp(s.z); break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::ld_mff00_c_a() {
  switch (s.op_tick) {
    case 7: WriteU8(0xff00 + s.c, s.a); break;
    case 8: s.op_tick = 0; break;
  }
}

void GB::ld_mff00_n_a() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: WriteU8(0xff00 + s.z, s.a); break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::ld_mhl_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: WriteU8(s.hl, s.z); break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::ld_mnn_a() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: s.w = ReadU8(s.pc++); break;
    case 15: WriteU8(s.wz, s.a); break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::ld_mnn_sp() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: s.w = ReadU8(s.pc++); break;
    case 15: WriteU8(s.wz + 1, s.sp >> 8); break;
    case 19: WriteU8(s.wz, s.sp); break;
    case 20: s.op_tick = 0; break;
  }
}

void GB::ld_mr_r(u16& mr, u8 r, int d) {
  switch (s.op_tick) {
    case 7: WriteU8(mr, r); mr += d; break;
    case 8: s.op_tick = 0; break;
  }
}

void GB::ld_r_mr(u8& r, u16& mr, int d) {
  switch (s.op_tick) {
    case 7: r = ReadU8(mr); mr += d; break;
    case 8: s.op_tick = 0; break;
  }
}

void GB::ld_r_n(u8& r) {
  switch (s.op_tick) {
    case 7: r = ReadU8(s.pc++); break;
    case 8: s.op_tick = 0; break;
  }
}

void GB::ld_rr_nn(u16& rr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 11: s.w = ReadU8(s.pc++); break;
    case 12: rr = s.wz; s.op_tick = 0; break;
  }
}

void GB::ld_r_r(u8& rd, u8 rs) {
  switch (s.op_tick) {
    case 4: rd = rs; s.op_tick = 0; break;
  }
}

void GB::ld_sp_hl() {
  switch (s.op_tick) {
    case 4: s.sp = s.hl; break;
    case 8: s.op_tick = 0; break;
  }
}

void GB::nop() {
  switch (s.op_tick) {
    case 4: s.op_tick = 0; break;
  }
}

void GB::or_(u8 x) {
  s.a |= x;
  s.f = zflag(s.a);
}

void GB::or_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 8: or_(s.z); s.op_tick = 0; break;
  }
}

void GB::or_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: or_(s.z); s.op_tick = 0; break;
  }
}

void GB::or_r(u8 r) {
  switch (s.op_tick) {
    case 4: or_(r); s.op_tick = 0; break;
  }
}

void GB::pop_af() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.sp++); break;
    case 11: s.w = ReadU8(s.sp++); break;
    case 12: s.af = s.wz & 0xfff0; s.op_tick = 0; break;
  }
}

void GB::pop_rr(u16& rr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.sp++); break;
    case 11: s.w = ReadU8(s.sp++); break;
    case 12: rr = s.wz; s.op_tick = 0; break;
  }
}

void GB::push_rr(u16 rr) {
  switch (s.op_tick) {
    case 11: WriteU8(--s.sp, rr >> 8); break;
    case 15: WriteU8(--s.sp, rr); break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::res_r(int n, u8& r) {
  switch (s.op_tick) {
    case 8: r &= ~(1 << n); s.op_tick = 0; break;
  }
}

void GB::res_mr(int n, u16 mr) {
  switch (s.op_tick) {
    case 11: s.z = ReadU8(mr); break;
    case 15: WriteU8(mr, s.z & ~(1 << n)); break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::ret() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.sp++); break;
    case 11: s.w = ReadU8(s.sp++); break;
    case 12: s.pc = s.wz; break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::ret_f(u8 mask, u8 val) {
  switch (s.op_tick) {
    case 8: f_is(mask, val); break;
    case 11: s.z = ReadU8(s.sp++); break;
    case 15: s.w = ReadU8(s.sp++); break;
    case 16: s.pc = s.wz; break;
    case 20: s.op_tick = 0; break;
  }
}

void GB::reti() {
  switch (s.op_tick) {
    case 4: s.ime_enable = false; s.ime = true; break;
    case 7: s.z = ReadU8(s.sp++); break;
    case 11: s.w = ReadU8(s.sp++); break;
    case 12: s.pc = s.wz; break;
    case 16: s.op_tick = 0; break;
  }
}

u8 GB::rl(u8 x) {
  u8 c = (x >> 3) & 0x10;
  u8 r = (x << 1) | ((s.f >> 4) & 1);
  s.f = zflag(r) | c;
  return r;
}

u8 GB::rlc(u8 x) {
  u8 c = (x >> 3) & 0x10;
  u8 r = (x << 1) | (c >> 4);
  s.f = zflag(r) | c;
  return r;
}

void GB::rla() {
  switch (s.op_tick) {
    case 4: s.a = rl(s.a); s.f &= ~0x80; s.op_tick = 0; break;
  }
}

void GB::rlca() {
  switch (s.op_tick) {
    case 4: s.a = rlc(s.a); s.f &= ~0x80; s.op_tick = 0; break;
  }
}

void GB::rlc_r(u8& r) {
  switch (s.op_tick) {
    case 8: r = rlc(r); s.op_tick = 0; break;
  }
}

void GB::rlc_mr(u16 mr) {
  switch (s.op_tick) {
    case 11: s.z = ReadU8(mr); break;
    case 15: WriteU8(mr, rlc(s.z)); break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::rl_r(u8& r) {
  switch (s.op_tick) {
    case 8: r = rl(r); s.op_tick = 0; break;
  }
}

void GB::rl_mr(u16 mr) {
  switch (s.op_tick) {
    case 11: s.z = ReadU8(mr); break;
    case 15: WriteU8(mr, rl(s.z)); break;
    case 16: s.op_tick = 0; break;
  }
}

u8 GB::rr(u8 x) {
  u8 c = (x << 4) & 0x10;
  u8 r = ((s.f << 3) & 0x80) | (x >> 1);
  s.f = zflag(r) | c;
  return r;
}

u8 GB::rrc(u8 x) {
  u8 c = (x << 4) & 0x10;
  u8 r = (c << 3) | (x >> 1);
  s.f = zflag(r) | c;
  return r;
}

void GB::rra() {
  switch (s.op_tick) {
    case 4: s.a = rr(s.a); s.f &= ~0x80; s.op_tick = 0; break;
  }
}

void GB::rrca() {
  switch (s.op_tick) {
    case 4: s.a = rrc(s.a); s.f &= ~0x80; s.op_tick = 0; break;
  }
}

void GB::rrc_r(u8& r) {
  switch (s.op_tick) {
    case 8: r = rrc(r); s.op_tick = 0; break;
  }
}

void GB::rrc_mr(u16 mr) {
  switch (s.op_tick) {
    case 11: s.z = ReadU8(mr); break;
    case 15: WriteU8(mr, rrc(s.z)); break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::rr_r(u8& r) {
  switch (s.op_tick) {
    case 8: r = rr(r); s.op_tick = 0; break;
  }
}

void GB::rr_mr(u16 mr) {
  switch (s.op_tick) {
    case 11: s.z = ReadU8(mr); break;
    case 15: WriteU8(mr, rr(s.z)); break;
    case 16: s.op_tick = 0; break;
  }
}

void GB::rst(u8 n) {
  switch (s.op_tick) {
    case 11: WriteU8(--s.sp, s.pc >> 8); break;
    case 15: WriteU8(--s.sp, s.pc); s.pc = n; break;
    case 16: s.op_tick = 0; break;
  }
}

u8 GB::sub(u8 x, u8 c) {
  int r = s.a - x - c;
  s.f = zflag(r) | 0x40 | hflag(r, s.a, x) | ((r >> 4) & 0x10);
  return r;
}

void GB::sbc_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 8: s.a = sub(s.z, (s.f >> 4) & 1); s.op_tick = 0; break;
  }
}

void GB::sbc_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: s.a = sub(s.z, (s.f >> 4) & 1); s.op_tick = 0; break;
  }
}

void GB::sbc_r(u8 r) {
  switch (s.op_tick) {
    case 4: s.a = sub(r, (s.f >> 4) & 1); s.op_tick = 0; break;
  }
}

void GB::scf() {
  switch (s.op_tick) {
    case 4: s.f = (s.f & 0x80) | 0x10; s.op_tick = 0; break;
  }
}

void GB::set_r(int n, u8& r) {
  switch (s.op_tick) {
    case 8: r |= (1 << n); s.op_tick = 0; break;
  }
}

void GB::set_mr(int n, u16 mr) {
  switch (s.op_tick) {
    case 11: s.z = ReadU8(mr); break;
    case 15: WriteU8(mr, s.z | (1 << n)); break;
    case 16: s.op_tick = 0; break;
  }
}

u8 GB::sla(u8 x) {
  u8 c = (x >> 3) & 0x10;
  u8 r = x << 1;
  s.f = zflag(r) | c;
  return r;
}

void GB::sla_r(u8& r) {
  switch (s.op_tick) {
    case 8: r = sla(r); s.op_tick = 0; break;
  }
}

void GB::sla_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 11: WriteU8(mr, sla(s.z)); break;
    case 12: s.op_tick = 0; break;
  }
}

u8 GB::sra(u8 x) {
  u8 c = (x << 4) & 0x10;
  u8 r = (u8)(s8(x) >> 1);
  s.f = zflag(r) | c;
  return r;
}

void GB::sra_r(u8& r) {
  switch (s.op_tick) {
    case 8: r = sra(r); s.op_tick = 0; break;
  }
}

void GB::sra_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 11: WriteU8(mr, sra(s.z)); break;
    case 12: s.op_tick = 0; break;
  }
}

u8 GB::srl(u8 x) {
  u8 c = (x << 4) & 0x10;
  u8 r = x >> 1;
  s.f = zflag(r) | c;
  return r;
}

void GB::srl_r(u8& r) {
  switch (s.op_tick) {
    case 8: r = srl(r); s.op_tick = 0; break;
  }
}

void GB::srl_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 11: WriteU8(mr, srl(s.z)); break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::stop() {}

void GB::sub_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 8: s.a = sub(s.z, 0); s.op_tick = 0; break;
  }
}

void GB::sub_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: s.a = sub(s.z, 0); s.op_tick = 0; break;
  }
}

void GB::sub_r(u8 r) {
  switch (s.op_tick) {
    case 4: s.a = sub(r, 0); s.op_tick = 0; break;
  }
}

u8 GB::swap(u8 x) {
  u8 r = (x << 4) | (x >> 4);
  s.f = zflag(r);
  return r;
}

void GB::swap_r(u8& r) {
  switch (s.op_tick) {
    case 8: r = swap(r); s.op_tick = 0; break;
  }
}

void GB::swap_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 11: WriteU8(mr, swap(s.z)); break;
    case 12: s.op_tick = 0; break;
  }
}

void GB::xor_(u8 x) {
  s.a ^= x;
  s.f = zflag(s.a);
}

void GB::xor_mr(u16 mr) {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(mr); break;
    case 8: xor_(s.z); s.op_tick = 0; break;
  }
}

void GB::xor_n() {
  switch (s.op_tick) {
    case 7: s.z = ReadU8(s.pc++); break;
    case 8: xor_(s.z); s.op_tick = 0; break;
  }
}

void GB::xor_r(u8 r) {
  switch (s.op_tick) {
    case 4: xor_(r); s.op_tick = 0; break;
  }
}

void GB::StepPPU() {
  if (!(s.io[LCDC] & 0x80)) {
    return;
  }
  u8& if_ = s.io[IF];
  u8& stat = s.io[STAT];
  u8& ly = s.io[LY];

  // TODO(binji): proper timing.
  s.ppu_line_tick++;
  if (ly < 144) {
    if (s.ppu_line_tick == 80) {
      stat = (stat & ~3) | 3;
    } else if (s.ppu_line_tick == 80 + 172) {
      stat = (stat & ~3) | 0;
      if (stat & 0x08) { if_ |= 2; }
    }
  }
  if (s.ppu_line_tick == 456) {
    switch (++ly) {
      case 144:
        stat = (stat & ~3) | 1;
        if (stat & 0x10) { if_ |= 2; }
        if_ |= 1;
        break;
      case 154:
        ly = 0;
        // Fallthrough.
      default:
        stat = (stat & ~3) | 2;
        if (stat & 0x20) { if_ |= 2; }
        break;
    }
    if ((stat & 0x40) && ly == s.io[LYC]) { if_ |= 2; }
    s.ppu_line_tick = 0;
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

static void sprint_hex(char* buffer, u8 val) {
  const char hex_digits[] = "0123456789abcdef";
  buffer[0] = hex_digits[(val >> 4) & 0xf];
  buffer[1] = hex_digits[val & 0xf];
}

int GB::Disassemble(u16 addr, char* buffer, size_t size) {
  char temp[64];
  char bytes[][3] = {"  ", "  "};
  const char* mnemonic = "*INVALID*";

  u8 opcode = ReadU8(addr);
  u8 num_bytes = s_opcode_bytes[opcode];
  switch (num_bytes) {
    case 0: break;
    case 1: mnemonic = s_opcode_mnemonic[opcode]; break;
    case 2: {
      u8 byte = ReadU8(addr + 1);
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
      u8 byte1 = ReadU8(addr + 1);
      u8 byte2 = ReadU8(addr + 2);
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
  if (s.op_tick == 0) {
    printf("a:%02x f:%c%c%c%c bc:%04x de:%04x hl:%04x sp:%04x pc:%04x", s.a,
           (s.f & 0x80) ? 'z' : '-', (s.f & 0x40) ? 'n' : '-',
           (s.f & 0x20) ? 'h' : '-', (s.f & 0x10) ? 'c' : '-', s.bc, s.de, s.hl,
           s.sp, s.pc);
#if 0
    printf(" (t: %2" PRIu64"/%10" PRIu64 ")", s.op_tick, s.tick);
#else
    printf(" (cy: %" PRIu64 ")", s.tick);
#endif
    printf(" |");
    PrintInstruction(s.pc);
    printf("\n");
  }
}


Buffer ReadFile(const char* filename) {
  std::ifstream file(filename, std::ios::binary | std::ios::ate);
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

int main(int argc, char** argv) {
  try {
    if (argc < 2) {
      std::cout << "No rom file given.\n";
      return 1;
    }

    GB gb(ReadFile(argv[1]), Variant::Guess);

    for (int i = 0; i < 3565492; ++i) {
      gb.Trace();
      gb.StepCPU();
      gb.StepPPU();
    }

    return 0;
  } catch (const Error& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
