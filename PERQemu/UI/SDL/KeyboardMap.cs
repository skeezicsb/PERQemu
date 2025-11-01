//
// KeyboardMap.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PERQemu is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with PERQemu.  If not, see <http://www.gnu.org/licenses/>.
//

using SDL2;

using System;
using System.Collections.Generic;

using PERQemu.Config;

namespace PERQemu.UI
{
    public enum KeyCode
    {
        // Special keys
        Help, Oops, LineFeed,

        // Alphanumerics
        A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
        Pad0, Pad1, Pad2, Pad3, Pad4, Pad5, Pad6, Pad7, Pad8, Pad9,

        // Punctuation and symbols
        Minus, Equals, BackQuote, LeftBracket, RightBracket, BackSlash, SemiColon, Quote,
        Comma, Period, Slash, Space, BackSpace, AccEsc, RejDel, Tab, Return,

        // PERQ-2 keys (and some pseudo keys)
        Setup, Break, NoScroll, PF1, PF2, PF3, PF4, Left, Right, Up, Down, Enter,
        PadComma, PadPeriod, PadPlus, PadMinus, PadMultiply, PadDivide, PadEquals
    }

    public struct PERQKey
    {
        public PERQKey(KeyCode code, byte value)
        {
            Code = code;
            Normal = Shift = Control = CtrlShift = value;
        }

        /// <summary>
        /// Shortcut for the PERQ-1 keyboard: control always just sets the high bit.
        /// </summary>
        public PERQKey(KeyCode code, byte normal, byte shifted)
        {
            Code = code;
            Normal = normal;
            Shift = shifted;
            Control = (byte)(normal | 0x80);
            CtrlShift = (byte)(shifted | 0x80);
        }

        /// <summary>
        /// Fully specify a key.
        /// </summary>
        public PERQKey(KeyCode code, byte normal, byte shifted, byte ctrl, byte ctrlShifted)
        {
            Code = code;
            Normal = normal;
            Shift = shifted;
            Control = ctrl;
            CtrlShift = ctrlShifted;
        }

        public KeyCode Code;
        public byte Normal;
        public byte Shift;
        public byte Control;
        public byte CtrlShift;
    }

    /// <summary>
    /// Maps the SDL keyboard codes to the PERQ ASCII representation.
    /// </summary>
    public sealed class KeyboardMap
    {
        public KeyboardMap(ChassisType machine)
        {
            _map = new Dictionary<SDL.SDL_Keycode, PERQKey>();

            if (machine == ChassisType.PERQ1)
                SetupPERQ1Map();
            else
                SetupPERQ2Map();

            // Set the initial state of the CAPS LOCK and NUM LOCK keys,
            // based on the Console's setting?
            _lockCaps = Console.CapsLock;
            _lockNums = Console.NumberLock;
        }

        public bool CapsLock => _lockCaps;
        public bool NumLock => _lockNums;

        // Toggle the state of a given "lock" key
        public void SetLockKeyState(SDL.SDL_Keycode keycode)
        {
            switch (keycode)
            {
                case SDL.SDL_Keycode.SDLK_CAPSLOCK:
                    _lockCaps = !_lockCaps;
                    Log.Debug(Category.Keyboard, "CAPS LOCK is {0}", _lockCaps);
                    break;

                case SDL.SDL_Keycode.SDLK_NUMLOCKCLEAR:
                    _lockNums = !_lockNums;
                    Log.Debug(Category.Keyboard, "NUM LOCK is {0}", _lockNums);
                    break;
            }
        }

        public void SetKeyMapping(SDL.SDL_Keycode keycode, PERQKey key)
        {
#if DEBUG
            if (_map.ContainsKey(keycode))
            {
                Console.WriteLine($"Duplicate keycode {keycode} (key {key}) entered in map!");
            }
#endif
            _map[keycode] = key;
        }

        public byte GetKeyValue(SDL.SDL_Keycode keycode, bool shift, bool control)
        {
            // Is it a key we recognize?
            if (!_map.ContainsKey(keycode)) return 0;

            // Yep!
            var perqKey = _map[keycode];

            // Account for the state of CAPS LOCK.  On a real PERQ 1, CAPS LOCK is
            // _really_ a SHIFT LOCK -- applies to every key, not just letters!  On
            // the PERQ 2, and here, we'll do the more expected CAPS LOCK behavior...
            shift |= ((keycode >= SDL.SDL_Keycode.SDLK_a && keycode <= SDL.SDL_Keycode.SDLK_z) && _lockCaps);

            // Okay, yes, I may be a little bit deranged
            return (shift ? (control ? perqKey.CtrlShift : perqKey.Shift) :
                            (control ? perqKey.Control : perqKey.Normal));
        }

        /// <summary>
        /// Map the custom PERQ-1 keyboard.
        /// </summary>
        void SetupPERQ1Map()
        {
            // Alphabetic
            SetKeyMapping(SDL.SDL_Keycode.SDLK_a, new PERQKey(KeyCode.A, 0x61, 0x41));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_b, new PERQKey(KeyCode.B, 0x62, 0x42));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_c, new PERQKey(KeyCode.C, 0x63, 0x43));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_d, new PERQKey(KeyCode.D, 0x64, 0x44));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_e, new PERQKey(KeyCode.E, 0x65, 0x45));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_f, new PERQKey(KeyCode.F, 0x66, 0x46));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_g, new PERQKey(KeyCode.G, 0x67, 0x47));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_h, new PERQKey(KeyCode.H, 0x68, 0x48));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_i, new PERQKey(KeyCode.I, 0x69, 0x49));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_j, new PERQKey(KeyCode.J, 0x6a, 0x4a));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_k, new PERQKey(KeyCode.K, 0x6b, 0x4b));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_l, new PERQKey(KeyCode.L, 0x6c, 0x4c));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_m, new PERQKey(KeyCode.M, 0x6d, 0x4d));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_n, new PERQKey(KeyCode.N, 0x6e, 0x4e));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_o, new PERQKey(KeyCode.O, 0x6f, 0x4f));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_p, new PERQKey(KeyCode.P, 0x70, 0x50));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_q, new PERQKey(KeyCode.Q, 0x71, 0x51));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_r, new PERQKey(KeyCode.R, 0x72, 0x52));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_s, new PERQKey(KeyCode.S, 0x73, 0x53));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_t, new PERQKey(KeyCode.T, 0x74, 0x54));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_u, new PERQKey(KeyCode.U, 0x75, 0x55));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_v, new PERQKey(KeyCode.V, 0x76, 0x56));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_w, new PERQKey(KeyCode.W, 0x77, 0x57));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_x, new PERQKey(KeyCode.X, 0x78, 0x58));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_y, new PERQKey(KeyCode.Y, 0x79, 0x59));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_z, new PERQKey(KeyCode.Z, 0x7a, 0x5a));

            // Numeric
            SetKeyMapping(SDL.SDL_Keycode.SDLK_1, new PERQKey(KeyCode.Num1, 0x31, 0x21));       // numeral 1, '!'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_2, new PERQKey(KeyCode.Num2, 0x32, 0x40));       // numeral 2, '@'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_3, new PERQKey(KeyCode.Num3, 0x33, 0x23));       // numeral 3, '#'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_4, new PERQKey(KeyCode.Num4, 0x34, 0x24));       // numeral 4, '$'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_5, new PERQKey(KeyCode.Num5, 0x35, 0x25));       // numeral 5, '%'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_6, new PERQKey(KeyCode.Num6, 0x36, 0x5e));       // numeral 6, '^'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_7, new PERQKey(KeyCode.Num7, 0x37, 0x26));       // numeral 7, '%'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_8, new PERQKey(KeyCode.Num8, 0x38, 0x2a));       // numeral 8, '*'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_9, new PERQKey(KeyCode.Num9, 0x39, 0x28));       // numeral 9, '('
            SetKeyMapping(SDL.SDL_Keycode.SDLK_0, new PERQKey(KeyCode.Num0, 0x30, 0x29));       // numeral 0, ')'

            // Punctuation
            SetKeyMapping(SDL.SDL_Keycode.SDLK_MINUS, new PERQKey(KeyCode.Minus, 0x2d, 0x5f));              // '-', '_'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_EQUALS, new PERQKey(KeyCode.Equals, 0x3d, 0x2b));            // '=', '+'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKQUOTE, new PERQKey(KeyCode.BackQuote, 0x60, 0x7e));      // '`', '~'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_LEFTBRACKET, new PERQKey(KeyCode.LeftBracket, 0x5b, 0x7b));  // '[', '{'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RIGHTBRACKET, new PERQKey(KeyCode.RightBracket, 0x5d, 0x7d));// ']', '}'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKSLASH, new PERQKey(KeyCode.BackSlash, 0x5c, 0x7c));      // '\', '|'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SEMICOLON, new PERQKey(KeyCode.SemiColon, 0x3b, 0x3a));      // ';', ':'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_QUOTE, new PERQKey(KeyCode.Quote, 0x27, 0x22));              // "'", '"'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_COMMA, new PERQKey(KeyCode.Comma, 0x2c, 0x3c));              // ',', '<'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_PERIOD, new PERQKey(KeyCode.Period, 0x2e, 0x3e));            // '.', '>'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SLASH, new PERQKey(KeyCode.Slash, 0x2f, 0x3f));              // '/', '?'

            // Editing keys
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SPACE, new PERQKey(KeyCode.Space, 0x20, 0x20));          // SPACE
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKSPACE, new PERQKey(KeyCode.BackSpace, 0x08, 0x08));  // BACKSPACE
            SetKeyMapping(SDL.SDL_Keycode.SDLK_TAB, new PERQKey(KeyCode.Tab, 0x09, 0x09));              // TAB (no back tab on PERQ)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_ESCAPE, new PERQKey(KeyCode.AccEsc, 0x1b, 0x1b));        // ESCAPE
            SetKeyMapping(SDL.SDL_Keycode.SDLK_INSERT, new PERQKey(KeyCode.AccEsc, 0x1b, 0x1b));        // INSERT (same as ESC on PERQ)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_DELETE, new PERQKey(KeyCode.RejDel, 0x7f, 0x7f));        // DELETE
            SetKeyMapping(SDL.SDL_Keycode.SDLK_DOWN, new PERQKey(KeyCode.LineFeed, 0x0a, 0x0a));        // LINEFEED
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RETURN, new PERQKey(KeyCode.Return, 0x0d, 0x0d));        // ENTER

            // Function keys
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F1, new PERQKey(KeyCode.Help, 0x07, 0x07));              // HELP (^g)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_HELP, new PERQKey(KeyCode.Help, 0x07, 0x07));            // HELP (^g)    (Mac)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F2, new PERQKey(KeyCode.Oops, 0x15, 0x15));              // OOPS (^u)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_CLEAR, new PERQKey(KeyCode.Oops, 0x15, 0x15));           // OOPS (^u)    (Mac)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F3, new PERQKey(KeyCode.LineFeed, 0x0a, 0x0a));          // LINEFEED (^j)

            Log.Debug(Category.Keyboard, "PERQ-1 map initialized");
        }

        /// <summary>
        /// Map the PERQ 2 "VT100-style" keyboard.
        /// NB: This keyboard sends the data inverted!
        /// </summary>
        void SetupPERQ2Map()
        {
            // Alphabetic
            SetKeyMapping(SDL.SDL_Keycode.SDLK_a, new PERQKey(KeyCode.A, 0x9e, 0xbe, 0x1e, 0x3e));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_b, new PERQKey(KeyCode.B, 0x9d, 0xbd, 0x1d, 0x3d));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_c, new PERQKey(KeyCode.C, 0x9c, 0xbc, 0x1c, 0x3c));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_d, new PERQKey(KeyCode.D, 0x9b, 0xbb, 0x1b, 0x3b));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_e, new PERQKey(KeyCode.E, 0x9a, 0xba, 0x1a, 0x3a));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_f, new PERQKey(KeyCode.F, 0x99, 0xb9, 0x19, 0x39));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_g, new PERQKey(KeyCode.G, 0x98, 0xb8, 0x18, 0x38));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_h, new PERQKey(KeyCode.H, 0x97, 0xb7, 0x17, 0x37));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_i, new PERQKey(KeyCode.I, 0x96, 0xb6, 0x16, 0x36));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_j, new PERQKey(KeyCode.J, 0x95, 0xb5, 0x15, 0x35));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_k, new PERQKey(KeyCode.K, 0x94, 0xb4, 0x14, 0x34));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_l, new PERQKey(KeyCode.L, 0x93, 0xb3, 0x13, 0x33));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_m, new PERQKey(KeyCode.M, 0x92, 0xb2, 0x12, 0x32));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_n, new PERQKey(KeyCode.N, 0x91, 0xb1, 0x11, 0x31));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_o, new PERQKey(KeyCode.O, 0x90, 0xb0, 0x10, 0x30));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_p, new PERQKey(KeyCode.P, 0x8f, 0xaf, 0x0f, 0x2f));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_q, new PERQKey(KeyCode.Q, 0x8e, 0xae, 0x0e, 0x2e));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_r, new PERQKey(KeyCode.R, 0x8d, 0xad, 0x0d, 0x2d));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_s, new PERQKey(KeyCode.S, 0x8c, 0xac, 0x0c, 0x2c));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_t, new PERQKey(KeyCode.T, 0x8b, 0xab, 0x0b, 0x2b));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_u, new PERQKey(KeyCode.U, 0x8a, 0xaa, 0x0a, 0x2a));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_v, new PERQKey(KeyCode.V, 0x89, 0xa9, 0x09, 0x29));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_w, new PERQKey(KeyCode.W, 0x88, 0xa8, 0x08, 0x28));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_x, new PERQKey(KeyCode.X, 0x87, 0xa7, 0x07, 0x27));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_y, new PERQKey(KeyCode.Y, 0x86, 0xa6, 0x06, 0x26));
            SetKeyMapping(SDL.SDL_Keycode.SDLK_z, new PERQKey(KeyCode.Z, 0x85, 0xa5, 0x05, 0x25));

            // Numeric
            SetKeyMapping(SDL.SDL_Keycode.SDLK_1, new PERQKey(KeyCode.Num1, 0xce, 0xde, 0x4e, 0x5e));    // numeral 1, '!'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_2, new PERQKey(KeyCode.Num2, 0xcd, 0xbf, 0x4d, 0x3f));    // numeral 2, '@'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_3, new PERQKey(KeyCode.Num3, 0xcc, 0xdc, 0x4c, 0x5c));    // numeral 3, '#'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_4, new PERQKey(KeyCode.Num4, 0xcb, 0xdb, 0x4b, 0x5b));    // numeral 4, '$'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_5, new PERQKey(KeyCode.Num5, 0xca, 0xda, 0x4a, 0x5a));    // numeral 5, '%'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_6, new PERQKey(KeyCode.Num6, 0xc9, 0xa1, 0x49, 0x21));    // numeral 6, '^'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_7, new PERQKey(KeyCode.Num7, 0xc8, 0xd9, 0x48, 0x59));    // numeral 7, '%'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_8, new PERQKey(KeyCode.Num8, 0xc7, 0xd5, 0x47, 0x55));    // numeral 8, '*'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_9, new PERQKey(KeyCode.Num9, 0xc6, 0xd7, 0x46, 0x57));    // numeral 9, '('
            SetKeyMapping(SDL.SDL_Keycode.SDLK_0, new PERQKey(KeyCode.Num0, 0xcf, 0xd6, 0x4f, 0x56));    // numeral 0, ')'

            // Punctuation
            SetKeyMapping(SDL.SDL_Keycode.SDLK_MINUS, new PERQKey(KeyCode.Minus, 0xd2, 0xa0, 0x52, 0x20));              // '-', '_'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_EQUALS, new PERQKey(KeyCode.Equals, 0xc2, 0xd4, 0x42, 0x54));            // '=', '+'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKQUOTE, new PERQKey(KeyCode.BackQuote, 0x9f, 0x81, 0x1f, 0x01));      // '`', '~'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_LEFTBRACKET, new PERQKey(KeyCode.LeftBracket, 0xa4, 0x84, 0x24, 0x04));  // '[', '{'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RIGHTBRACKET, new PERQKey(KeyCode.RightBracket, 0xa2, 0x82, 0x22, 0x02));// ']', '}'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKSLASH, new PERQKey(KeyCode.BackSlash, 0xa3, 0x83, 0x23, 0x03));      // '\', '|'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SEMICOLON, new PERQKey(KeyCode.SemiColon, 0xc4, 0xc5, 0x44, 0x45));      // ';', ':'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_QUOTE, new PERQKey(KeyCode.Quote, 0xd8, 0xdd, 0x58, 0x5d));              // "'", '"'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_COMMA, new PERQKey(KeyCode.Comma, 0xd3, 0xc3, 0x53, 0x43));              // ',', '<'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_PERIOD, new PERQKey(KeyCode.Period, 0xd1, 0xc1, 0x51, 0x41));            // '.', '>'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SLASH, new PERQKey(KeyCode.Slash, 0xd0, 0xc0, 0x50, 0x40));              // '/', '?'

            // Editing keys
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SPACE, new PERQKey(KeyCode.Space, 0xdf));                            // SPACE
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKSPACE, new PERQKey(KeyCode.BackSpace, 0xf7, 0xf7, 0x77, 0x77));  // BACKSPACE
            SetKeyMapping(SDL.SDL_Keycode.SDLK_TAB, new PERQKey(KeyCode.Tab, 0xf6, 0xf6, 0x76, 0x76));              // TAB
            SetKeyMapping(SDL.SDL_Keycode.SDLK_ESCAPE, new PERQKey(KeyCode.AccEsc, 0xe4, 0xe4, 0x64, 0x64));        // ESCAPE
            SetKeyMapping(SDL.SDL_Keycode.SDLK_INSERT, new PERQKey(KeyCode.AccEsc, 0xe4, 0xe4, 0x64, 0x64));        // INSERT (same as ESC on PERQ)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_DELETE, new PERQKey(KeyCode.RejDel, 0x80, 0x80, 0xf4, 0xf4));        // DELETE (send SETUP, not NUL!)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RETURN, new PERQKey(KeyCode.Return, 0xf2, 0xf2, 0x72, 0x72));        // RETURN

            // Arrows
            SetKeyMapping(SDL.SDL_Keycode.SDLK_UP, new PERQKey(KeyCode.Up, 0x7f, 0x7f, 0xe3, 0xe3));                // UP
            SetKeyMapping(SDL.SDL_Keycode.SDLK_DOWN, new PERQKey(KeyCode.Down, 0x7e, 0x7e, 0xe2, 0xe2));            // DOWN
            SetKeyMapping(SDL.SDL_Keycode.SDLK_LEFT, new PERQKey(KeyCode.Left, 0x7d, 0x7d, 0xe1, 0xe1));            // LEFT 
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RIGHT, new PERQKey(KeyCode.Right, 0x7c, 0x7c, 0xe0, 0xe0));          // RIGHT

            // PERQ/VT100 keys
            SetKeyMapping(SDL.SDL_Keycode.SDLK_HELP, new PERQKey(KeyCode.Help, 0xf8, 0xf8, 0x78, 0x78));            // HELP
            SetKeyMapping(SDL.SDL_Keycode.SDLK_CLEAR, new PERQKey(KeyCode.Oops, 0xea, 0xea, 0x6a, 0x6a));           // OOPS
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_CLEAR, new PERQKey(KeyCode.Oops, 0xea, 0xea, 0x6a, 0x6a));        // OOPS (Mac)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SYSREQ, new PERQKey(KeyCode.Break, 0x7f, 0x7e, 0x7d, 0x7c));         // BREAK
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SCROLLLOCK, new PERQKey(KeyCode.NoScroll, 0xf3, 0xf3, 0xf1, 0xf1));  // NOSCRL
            SetKeyMapping(SDL.SDL_Keycode.SDLK_PAGEDOWN, new PERQKey(KeyCode.LineFeed, 0xf5, 0xf5, 0x75, 0x75));    // LINEFEED
            SetKeyMapping(SDL.SDL_Keycode.SDLK_PAGEUP, new PERQKey(KeyCode.Setup, 0xf4, 0xf4, 0xf9, 0xf9));         // SETUP

            // Keypad
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F1, new PERQKey(KeyCode.PF1, 0x7b));                 // PF1
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F2, new PERQKey(KeyCode.PF2, 0x7a));                 // PF2
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F3, new PERQKey(KeyCode.PF3, 0x79));                 // PF3
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F4, new PERQKey(KeyCode.PF4, 0x74));                 // PF4

            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_PERIOD, new PERQKey(KeyCode.PadPeriod, 0x6b));    // Keypad '.'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_MINUS, new PERQKey(KeyCode.PadMinus, 0x6c));      // Keypad '-'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_COMMA, new PERQKey(KeyCode.PadComma, 0x6d));      // Keypad ','
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_ENTER, new PERQKey(KeyCode.Enter, 0x73));         // ENTER

            // Not present on PERQ, map to expected key on modern keyboards:
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_PLUS, new PERQKey(KeyCode.PadPlus, 0xd4));        // '+'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_EQUALS, new PERQKey(KeyCode.PadEquals, 0xc2));    // '='
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_DIVIDE, new PERQKey(KeyCode.PadDivide, 0xd0));    // '/'
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_MULTIPLY, new PERQKey(KeyCode.PadMultiply, 0xd5));// '*'

            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_0, new PERQKey(KeyCode.Pad0, 0x69));              // NumPad 0
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_1, new PERQKey(KeyCode.Pad1, 0x68));              // NumPad 1
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_2, new PERQKey(KeyCode.Pad2, 0x67));              // NumPad 2
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_3, new PERQKey(KeyCode.Pad3, 0x66));              // NumPad 3
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_4, new PERQKey(KeyCode.Pad4, 0x65));              // NumPad 4
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_5, new PERQKey(KeyCode.Pad5, 0x63));              // NumPad 5
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_6, new PERQKey(KeyCode.Pad6, 0x62));              // NumPad 6
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_7, new PERQKey(KeyCode.Pad7, 0x61));              // NumPad 7
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_8, new PERQKey(KeyCode.Pad8, 0x60));              // NumPad 8
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_9, new PERQKey(KeyCode.Pad9, 0x5f));              // NumPad 9

            Log.Debug(Category.Keyboard, "PERQ-2 (VT100) map initialized");
        }

        /// <summary>
        /// Show the current keyboard mappings.  Quick and dirty, for now.
        /// </summary>
        public void PrintMap()
        {
            Console.WriteLine("Host key\tPERQ key");
            foreach (var hk in _map.Keys)
            {
                Console.WriteLine($"{hk.ToString()}\t{_map[hk].Code}");
            }
        }

        /// <summary>
        /// Remap a key entry to accommodate personal preference/different host keyboard layouts.            
        /// </summary>
        public void ReMap(KeyCode perqKey, SDL.SDL_Keycode hostKey)
        {
            // Todo: Write me! :-)
        }

        // One map to rule them all
        Dictionary<SDL.SDL_Keycode, PERQKey> _map;

        // Have to catch/test for CAPS LOCK status ourselves, and maintain local state.  Ugh.
        bool _lockCaps;
        bool _lockNums;
    }
}

