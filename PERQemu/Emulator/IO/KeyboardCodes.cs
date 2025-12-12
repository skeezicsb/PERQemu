//
// KeyboardCodes.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
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

using System;

using PERQemu.UI;

namespace PERQemu.IO
{
    /// <summary>
    /// Store the four possible values transmitted by a PERQ key as encoded
    /// in the keyboard PROMs.
    /// </summary>
    public struct PERQKey
    {
        /// <summary>
        /// For keys that don't encode any modifiers.
        /// </summary>
        public PERQKey(KeyCap name, byte value)
        {
            Name = name;
            Normal = Shift = Control = CtrlShift = value;
        }

        /// <summary>
        /// Shortcut for the PERQ-1 keyboard: control always just sets the high bit.
        /// </summary>
        public PERQKey(KeyCap name, byte normal, byte shifted)
        {
            Name = name;
            Normal = normal;
            Shift = shifted;
            Control = (byte)(normal | 0x80);
            CtrlShift = (byte)(shifted | 0x80);
        }

        /// <summary>
        /// Fully specify a key.
        /// </summary>
        public PERQKey(KeyCap name, byte normal, byte shifted, byte ctrl, byte ctrlShifted)
        {
            Name = name;
            Normal = normal;
            Shift = shifted;
            Control = ctrl;
            CtrlShift = ctrlShifted;
        }

        public KeyCap Name;
        public byte Normal;
        public byte Shift;
        public byte Control;
        public byte CtrlShift;
    }

    /// <summary>
    /// Static tables of PERQ "raw" 8-bit keyboard codes.
    /// </summary>
    public static class KeyboardCodes
    {
        static KeyboardCodes()
        {
            PERQ1Codes = new PERQKey[Enum.GetValues(typeof(KeyCap)).Length];
            PERQ2Codes = new PERQKey[Enum.GetValues(typeof(KeyCap)).Length];

            // PERQ 1 raw mappings
            PERQ1Codes[(int)KeyCap.A] = new PERQKey(KeyCap.A, 0x61, 0x41);
            PERQ1Codes[(int)KeyCap.B] = new PERQKey(KeyCap.B, 0x62, 0x42);
            PERQ1Codes[(int)KeyCap.C] = new PERQKey(KeyCap.C, 0x63, 0x43);
            PERQ1Codes[(int)KeyCap.D] = new PERQKey(KeyCap.D, 0x64, 0x44);
            PERQ1Codes[(int)KeyCap.E] = new PERQKey(KeyCap.E, 0x65, 0x45);
            PERQ1Codes[(int)KeyCap.F] = new PERQKey(KeyCap.F, 0x66, 0x46);
            PERQ1Codes[(int)KeyCap.G] = new PERQKey(KeyCap.G, 0x67, 0x47);
            PERQ1Codes[(int)KeyCap.H] = new PERQKey(KeyCap.H, 0x68, 0x48);
            PERQ1Codes[(int)KeyCap.I] = new PERQKey(KeyCap.I, 0x69, 0x49);
            PERQ1Codes[(int)KeyCap.J] = new PERQKey(KeyCap.J, 0x6a, 0x4a);
            PERQ1Codes[(int)KeyCap.K] = new PERQKey(KeyCap.K, 0x6b, 0x4b);
            PERQ1Codes[(int)KeyCap.L] = new PERQKey(KeyCap.L, 0x6c, 0x4c);
            PERQ1Codes[(int)KeyCap.M] = new PERQKey(KeyCap.M, 0x6d, 0x4d);
            PERQ1Codes[(int)KeyCap.N] = new PERQKey(KeyCap.N, 0x6e, 0x4e);
            PERQ1Codes[(int)KeyCap.O] = new PERQKey(KeyCap.O, 0x6f, 0x4f);
            PERQ1Codes[(int)KeyCap.P] = new PERQKey(KeyCap.P, 0x70, 0x50);
            PERQ1Codes[(int)KeyCap.Q] = new PERQKey(KeyCap.Q, 0x71, 0x51);
            PERQ1Codes[(int)KeyCap.R] = new PERQKey(KeyCap.R, 0x72, 0x52);
            PERQ1Codes[(int)KeyCap.S] = new PERQKey(KeyCap.S, 0x73, 0x53);
            PERQ1Codes[(int)KeyCap.T] = new PERQKey(KeyCap.T, 0x74, 0x54);
            PERQ1Codes[(int)KeyCap.U] = new PERQKey(KeyCap.U, 0x75, 0x55);
            PERQ1Codes[(int)KeyCap.V] = new PERQKey(KeyCap.V, 0x76, 0x56);
            PERQ1Codes[(int)KeyCap.W] = new PERQKey(KeyCap.W, 0x77, 0x57);
            PERQ1Codes[(int)KeyCap.X] = new PERQKey(KeyCap.X, 0x78, 0x58);
            PERQ1Codes[(int)KeyCap.Y] = new PERQKey(KeyCap.Y, 0x79, 0x59);
            PERQ1Codes[(int)KeyCap.Z] = new PERQKey(KeyCap.Z, 0x7a, 0x5a);

            PERQ1Codes[(int)KeyCap.Num1] = new PERQKey(KeyCap.Num1, 0x31, 0x21);        // numeral 1, '!'
            PERQ1Codes[(int)KeyCap.Num2] = new PERQKey(KeyCap.Num2, 0x32, 0x40);        // numeral 2, '@'
            PERQ1Codes[(int)KeyCap.Num3] = new PERQKey(KeyCap.Num3, 0x33, 0x23);        // numeral 3, '#'
            PERQ1Codes[(int)KeyCap.Num4] = new PERQKey(KeyCap.Num4, 0x34, 0x24);        // numeral 4, '$'
            PERQ1Codes[(int)KeyCap.Num5] = new PERQKey(KeyCap.Num5, 0x35, 0x25);        // numeral 5, '%'
            PERQ1Codes[(int)KeyCap.Num6] = new PERQKey(KeyCap.Num6, 0x36, 0x5e);        // numeral 6, '^'
            PERQ1Codes[(int)KeyCap.Num7] = new PERQKey(KeyCap.Num7, 0x37, 0x26);        // numeral 7, '%'
            PERQ1Codes[(int)KeyCap.Num8] = new PERQKey(KeyCap.Num8, 0x38, 0x2a);        // numeral 8, '*'
            PERQ1Codes[(int)KeyCap.Num9] = new PERQKey(KeyCap.Num9, 0x39, 0x28);        // numeral 9, '('
            PERQ1Codes[(int)KeyCap.Num0] = new PERQKey(KeyCap.Num0, 0x30, 0x29);        // numeral 0, ')'

            PERQ1Codes[(int)KeyCap.Minus] = new PERQKey(KeyCap.Minus, 0x2d, 0x5f);              // '-', '_'
            PERQ1Codes[(int)KeyCap.Equals] = new PERQKey(KeyCap.Equals, 0x3d, 0x2b);            // '=', '+'
            PERQ1Codes[(int)KeyCap.LeftBracket] = new PERQKey(KeyCap.LeftBracket, 0x5b, 0x7b);  // '[', '{'
            PERQ1Codes[(int)KeyCap.RightBracket] = new PERQKey(KeyCap.RightBracket, 0x5d, 0x7d);// ']', '}'
            PERQ1Codes[(int)KeyCap.Quote] = new PERQKey(KeyCap.Quote, 0x27, 0x22);              // "'", '"'
            PERQ1Codes[(int)KeyCap.BackQuote] = new PERQKey(KeyCap.BackQuote, 0x60, 0x7e);      // '`', '~'
            PERQ1Codes[(int)KeyCap.Slash] = new PERQKey(KeyCap.Slash, 0x2f, 0x3f);              // '/', '?'
            PERQ1Codes[(int)KeyCap.BackSlash] = new PERQKey(KeyCap.BackSlash, 0x5c, 0x7c);      // '\', '|'
            PERQ1Codes[(int)KeyCap.SemiColon] = new PERQKey(KeyCap.SemiColon, 0x3b, 0x3a);      // ';', ':'
            PERQ1Codes[(int)KeyCap.Comma] = new PERQKey(KeyCap.Comma, 0x2c, 0x3c);              // ',', '<'
            PERQ1Codes[(int)KeyCap.Period] = new PERQKey(KeyCap.Period, 0x2e, 0x3e);            // '.', '>'

            PERQ1Codes[(int)KeyCap.Ins] = new PERQKey(KeyCap.Ins, 0x1b, 0x1b);
            PERQ1Codes[(int)KeyCap.Del] = new PERQKey(KeyCap.Del, 0x7f, 0x7f);
            PERQ1Codes[(int)KeyCap.Help] = new PERQKey(KeyCap.Help, 0x07, 0x07);
            PERQ1Codes[(int)KeyCap.Tab] = new PERQKey(KeyCap.Tab, 0x09, 0x09);
            PERQ1Codes[(int)KeyCap.Space] = new PERQKey(KeyCap.Space, 0x20, 0x20);
            PERQ1Codes[(int)KeyCap.BackSpace] = new PERQKey(KeyCap.BackSpace, 0x08, 0x08);
            PERQ1Codes[(int)KeyCap.Oops] = new PERQKey(KeyCap.Oops, 0x15, 0x15);
            PERQ1Codes[(int)KeyCap.Return] = new PERQKey(KeyCap.Return, 0x0d, 0x0d);
            PERQ1Codes[(int)KeyCap.LineFeed] = new PERQKey(KeyCap.LineFeed, 0x0a, 0x0a);

            // PERQ 2 raw mappings
            PERQ2Codes[(int)KeyCap.A] = new PERQKey(KeyCap.A, 0x9e, 0xbe, 0x1e, 0x3e);
            PERQ2Codes[(int)KeyCap.B] = new PERQKey(KeyCap.B, 0x9d, 0xbd, 0x1d, 0x3d);
            PERQ2Codes[(int)KeyCap.C] = new PERQKey(KeyCap.C, 0x9c, 0xbc, 0x1c, 0x3c);
            PERQ2Codes[(int)KeyCap.D] = new PERQKey(KeyCap.D, 0x9b, 0xbb, 0x1b, 0x3b);
            PERQ2Codes[(int)KeyCap.E] = new PERQKey(KeyCap.E, 0x9a, 0xba, 0x1a, 0x3a);
            PERQ2Codes[(int)KeyCap.F] = new PERQKey(KeyCap.F, 0x99, 0xb9, 0x19, 0x39);
            PERQ2Codes[(int)KeyCap.G] = new PERQKey(KeyCap.G, 0x98, 0xb8, 0x18, 0x38);
            PERQ2Codes[(int)KeyCap.H] = new PERQKey(KeyCap.H, 0x97, 0xb7, 0x17, 0x37);
            PERQ2Codes[(int)KeyCap.I] = new PERQKey(KeyCap.I, 0x96, 0xb6, 0x16, 0x36);
            PERQ2Codes[(int)KeyCap.J] = new PERQKey(KeyCap.J, 0x95, 0xb5, 0x15, 0x35);
            PERQ2Codes[(int)KeyCap.K] = new PERQKey(KeyCap.K, 0x94, 0xb4, 0x14, 0x34);
            PERQ2Codes[(int)KeyCap.L] = new PERQKey(KeyCap.L, 0x93, 0xb3, 0x13, 0x33);
            PERQ2Codes[(int)KeyCap.M] = new PERQKey(KeyCap.M, 0x92, 0xb2, 0x12, 0x32);
            PERQ2Codes[(int)KeyCap.N] = new PERQKey(KeyCap.N, 0x91, 0xb1, 0x11, 0x31);
            PERQ2Codes[(int)KeyCap.O] = new PERQKey(KeyCap.O, 0x90, 0xb0, 0x10, 0x30);
            PERQ2Codes[(int)KeyCap.P] = new PERQKey(KeyCap.P, 0x8f, 0xaf, 0x0f, 0x2f);
            PERQ2Codes[(int)KeyCap.Q] = new PERQKey(KeyCap.Q, 0x8e, 0xae, 0x0e, 0x2e);
            PERQ2Codes[(int)KeyCap.R] = new PERQKey(KeyCap.R, 0x8d, 0xad, 0x0d, 0x2d);
            PERQ2Codes[(int)KeyCap.S] = new PERQKey(KeyCap.S, 0x8c, 0xac, 0x0c, 0x2c);
            PERQ2Codes[(int)KeyCap.T] = new PERQKey(KeyCap.T, 0x8b, 0xab, 0x0b, 0x2b);
            PERQ2Codes[(int)KeyCap.U] = new PERQKey(KeyCap.U, 0x8a, 0xaa, 0x0a, 0x2a);
            PERQ2Codes[(int)KeyCap.V] = new PERQKey(KeyCap.V, 0x89, 0xa9, 0x09, 0x29);
            PERQ2Codes[(int)KeyCap.W] = new PERQKey(KeyCap.W, 0x88, 0xa8, 0x08, 0x28);
            PERQ2Codes[(int)KeyCap.X] = new PERQKey(KeyCap.X, 0x87, 0xa7, 0x07, 0x27);
            PERQ2Codes[(int)KeyCap.Y] = new PERQKey(KeyCap.Y, 0x86, 0xa6, 0x06, 0x26);
            PERQ2Codes[(int)KeyCap.Z] = new PERQKey(KeyCap.Z, 0x85, 0xa5, 0x05, 0x25);

            PERQ2Codes[(int)KeyCap.Num1] = new PERQKey(KeyCap.Num1, 0xce, 0xde, 0x4e, 0x5e);        // numeral 1, '!'
            PERQ2Codes[(int)KeyCap.Num2] = new PERQKey(KeyCap.Num2, 0xcd, 0xbf, 0x4d, 0x3f);        // numeral 2, '@'
            PERQ2Codes[(int)KeyCap.Num3] = new PERQKey(KeyCap.Num3, 0xcc, 0xdc, 0x4c, 0x5c);        // numeral 3, '#'
            PERQ2Codes[(int)KeyCap.Num4] = new PERQKey(KeyCap.Num4, 0xcb, 0xdb, 0x4b, 0x5b);        // numeral 4, '$'
            PERQ2Codes[(int)KeyCap.Num5] = new PERQKey(KeyCap.Num5, 0xca, 0xda, 0x4a, 0x5a);        // numeral 5, '%'
            PERQ2Codes[(int)KeyCap.Num6] = new PERQKey(KeyCap.Num6, 0xc9, 0xa1, 0x49, 0x21);        // numeral 6, '^'
            PERQ2Codes[(int)KeyCap.Num7] = new PERQKey(KeyCap.Num7, 0xc8, 0xd9, 0x48, 0x59);        // numeral 7, '%'
            PERQ2Codes[(int)KeyCap.Num8] = new PERQKey(KeyCap.Num8, 0xc7, 0xd5, 0x47, 0x55);        // numeral 8, '*'
            PERQ2Codes[(int)KeyCap.Num9] = new PERQKey(KeyCap.Num9, 0xc6, 0xd7, 0x46, 0x57);        // numeral 9, '('
            PERQ2Codes[(int)KeyCap.Num0] = new PERQKey(KeyCap.Num0, 0xcf, 0xd6, 0x4f, 0x56);        // numeral 0, ')'

            PERQ2Codes[(int)KeyCap.Minus] = new PERQKey(KeyCap.Minus, 0xd2, 0xa0, 0x52, 0x20);              // '-', '_'
            PERQ2Codes[(int)KeyCap.Equals] = new PERQKey(KeyCap.Equals, 0xc2, 0xd4, 0x42, 0x54);            // '=', '+'
            PERQ2Codes[(int)KeyCap.LeftBracket] = new PERQKey(KeyCap.LeftBracket, 0xa4, 0x84, 0x24, 0x04);  // '[', '{'
            PERQ2Codes[(int)KeyCap.RightBracket] = new PERQKey(KeyCap.RightBracket, 0xa2, 0x82, 0x22, 0x02);// ']', '}'
            PERQ2Codes[(int)KeyCap.Quote] = new PERQKey(KeyCap.Quote, 0xd8, 0xdd, 0x58, 0x5d);              // "'", '"'
            PERQ2Codes[(int)KeyCap.BackQuote] = new PERQKey(KeyCap.BackQuote, 0x9f, 0x81, 0x1f, 0x01);      // '`', '~'
            PERQ2Codes[(int)KeyCap.Slash] = new PERQKey(KeyCap.Slash, 0xd0, 0xc0, 0x50, 0x40);              // '/', '?'
            PERQ2Codes[(int)KeyCap.BackSlash] = new PERQKey(KeyCap.BackSlash, 0xa3, 0x83, 0x23, 0x03);      // '\', '|'
            PERQ2Codes[(int)KeyCap.SemiColon] = new PERQKey(KeyCap.SemiColon, 0xc4, 0xc5, 0x44, 0x45);      // ';', ':'
            PERQ2Codes[(int)KeyCap.Comma] = new PERQKey(KeyCap.Comma, 0xd3, 0xc3, 0x53, 0x43);              // ',', '<'
            PERQ2Codes[(int)KeyCap.Period] = new PERQKey(KeyCap.Period, 0xd1, 0xc1, 0x51, 0x41);            // '.', '>'

            PERQ2Codes[(int)KeyCap.Space] = new PERQKey(KeyCap.Space, 0xdf);
            PERQ2Codes[(int)KeyCap.BackSpace] = new PERQKey(KeyCap.BackSpace, 0xf7, 0xf7, 0x77, 0x77);
            PERQ2Codes[(int)KeyCap.Tab] = new PERQKey(KeyCap.Tab, 0xf6, 0xf6, 0x76, 0x76);
            PERQ2Codes[(int)KeyCap.Ins] = new PERQKey(KeyCap.Ins, 0xe4, 0xe4, 0x64, 0x64);
            PERQ2Codes[(int)KeyCap.Del] = new PERQKey(KeyCap.Del, 0x80, 0x80, 0xf4, 0xf4);
            PERQ2Codes[(int)KeyCap.Return] = new PERQKey(KeyCap.Return, 0xf2, 0xf2, 0x72, 0x72);

            PERQ2Codes[(int)KeyCap.Up] = new PERQKey(KeyCap.Up, 0x7f, 0x7f, 0xe3, 0xe3);
            PERQ2Codes[(int)KeyCap.Down] = new PERQKey(KeyCap.Down, 0x7e, 0x7e, 0xe2, 0xe2);
            PERQ2Codes[(int)KeyCap.Left] = new PERQKey(KeyCap.Left, 0x7d, 0x7d, 0xe1, 0xe1);
            PERQ2Codes[(int)KeyCap.Right] = new PERQKey(KeyCap.Right, 0x7c, 0x7c, 0xe0, 0xe0);

            PERQ2Codes[(int)KeyCap.Setup] = new PERQKey(KeyCap.Setup, 0xf4, 0xf4, 0xf9, 0xf9);
            PERQ2Codes[(int)KeyCap.Help] = new PERQKey(KeyCap.Help, 0xf8, 0xf8, 0x78, 0x78);
            PERQ2Codes[(int)KeyCap.Oops] = new PERQKey(KeyCap.Oops, 0xea, 0xea, 0x6a, 0x6a);
            PERQ2Codes[(int)KeyCap.Break] = new PERQKey(KeyCap.Break, 0x7f, 0x7e, 0x7d, 0x7c);
            PERQ2Codes[(int)KeyCap.NoScroll] = new PERQKey(KeyCap.NoScroll, 0xf3, 0xf3, 0xf1, 0xf1);
            PERQ2Codes[(int)KeyCap.LineFeed] = new PERQKey(KeyCap.LineFeed, 0xf5, 0xf5, 0x75, 0x75);

            PERQ2Codes[(int)KeyCap.PF1] = new PERQKey(KeyCap.PF1, 0x7b);
            PERQ2Codes[(int)KeyCap.PF2] = new PERQKey(KeyCap.PF2, 0x7a);
            PERQ2Codes[(int)KeyCap.PF3] = new PERQKey(KeyCap.PF3, 0x79);
            PERQ2Codes[(int)KeyCap.PF4] = new PERQKey(KeyCap.PF4, 0x74);

            PERQ2Codes[(int)KeyCap.Pad0] = new PERQKey(KeyCap.Pad0, 0x69);              // NumPad 0
            PERQ2Codes[(int)KeyCap.Pad1] = new PERQKey(KeyCap.Pad1, 0x68);              // NumPad 1
            PERQ2Codes[(int)KeyCap.Pad2] = new PERQKey(KeyCap.Pad2, 0x67);              // NumPad 2
            PERQ2Codes[(int)KeyCap.Pad3] = new PERQKey(KeyCap.Pad3, 0x66);              // NumPad 3
            PERQ2Codes[(int)KeyCap.Pad4] = new PERQKey(KeyCap.Pad4, 0x65);              // NumPad 4
            PERQ2Codes[(int)KeyCap.Pad5] = new PERQKey(KeyCap.Pad5, 0x63);              // NumPad 5
            PERQ2Codes[(int)KeyCap.Pad6] = new PERQKey(KeyCap.Pad6, 0x62);              // NumPad 6
            PERQ2Codes[(int)KeyCap.Pad7] = new PERQKey(KeyCap.Pad7, 0x61);              // NumPad 7
            PERQ2Codes[(int)KeyCap.Pad8] = new PERQKey(KeyCap.Pad8, 0x60);              // NumPad 8
            PERQ2Codes[(int)KeyCap.Pad9] = new PERQKey(KeyCap.Pad9, 0x5f);              // NumPad 9

            PERQ2Codes[(int)KeyCap.PadPeriod] = new PERQKey(KeyCap.PadPeriod, 0x6b);    // Keypad '.'
            PERQ2Codes[(int)KeyCap.PadMinus] = new PERQKey(KeyCap.PadMinus, 0x6c);      // Keypad '-'
            PERQ2Codes[(int)KeyCap.PadComma] = new PERQKey(KeyCap.PadComma, 0x6d);      // Keypad ','
            PERQ2Codes[(int)KeyCap.Enter] = new PERQKey(KeyCap.Enter, 0x73);            // ENTER

            // Define the pseudo keys so that modern keypads send something appropriate
            // by default.  The odd VT100-style pad on the PERQ doesn't have these!
            PERQ2Codes[(int)KeyCap.PadPlus] = new PERQKey(KeyCap.PadPlus, 0xd4);        // '+'
            PERQ2Codes[(int)KeyCap.PadEquals] = new PERQKey(KeyCap.PadEquals, 0xc2);    // '='
            PERQ2Codes[(int)KeyCap.PadDivide] = new PERQKey(KeyCap.PadDivide, 0xd0);    // '/'
            PERQ2Codes[(int)KeyCap.PadMultiply] = new PERQKey(KeyCap.PadMultiply, 0xd5);// '*'
        }

        public static readonly PERQKey[] PERQ1Codes;
        public static readonly PERQKey[] PERQ2Codes;
    }
}
