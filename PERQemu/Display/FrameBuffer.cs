// framebuffer.cs - Copyright 2018-2019 S. Boondoggle (skeezicsb@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PERQemu is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with PERQemu.  If not, see <http://www.gnu.org/licenses/>.
//

using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;
using System.Runtime.InteropServices;

namespace PERQemu.Display
{
    /// <summary>
    /// A bitmap pinned in memory that can be directly painted on screen
    /// without copying.  This allegedly makes the garbage collector unhappy
    /// since it can't coalesce free space around the pinned buffers, but it
    /// does allow the display to go like a bat out of hell, even under plain
    /// old crappy WinForms.  Mono's libgdiplus, however, insists on copying
    /// and freeing/disposing some underlying storage no matter what strategy
    /// is used, so it's a losing proposition no matter what.  At least this
    /// way we lose and go fast, instead of just losing slowly.  Yay.
    /// </summary>
    public class DirectBitmap : IDisposable
    {

        public DirectBitmap(int width, int height, int stride, PixelFormat fmt)
        {
            Width = width;
            Height = height;
            Bits = new ulong[(width / 64) * height];
            BitsHandle = GCHandle.Alloc(Bits, GCHandleType.Pinned);
            Bitmap = new Bitmap(width, height, stride, fmt, BitsHandle.AddrOfPinnedObject());
        }

        public int Width { get; private set; }
        public int Height { get; private set; }
        public ulong[] Bits { get; private set; }
        public Bitmap Bitmap { get; private set; }
        public bool Disposed { get; private set; }

        protected GCHandle BitsHandle { get; private set; }

        public void SetQuad(int addr, ulong q)
        {
            Bits[addr] = q;
        }

        public ulong GetQuad(int addr)
        {
            return Bits[addr];
        }

        public void Dispose()
        {
            Console.WriteLine("DirectBitmap dispose {0}", Disposed);
            if (Disposed) return;
            Disposed = true;
            Bitmap.Dispose();
            BitsHandle.Free();
        }
    }

    /// <summary>
    /// A double-buffered control that swaps two DirectBitmaps.  After much
    /// testing and study, this is the best balance between fast and ugly...
    /// </summary>
    class FrameBufferControl : PictureBox
    {

        public FrameBufferControl(Size size)
        {
            this.SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.Opaque, true);
            this.SetStyle(ControlStyles.OptimizedDoubleBuffer, false);

            this.ClientSize = size;

            // Allocate our two fixed Bitmaps for a front and back buffer.  At
            // some point maybe we'll have a third grey-scale buffer for when the
            // display is "off" -- simulating the rolling retrace when the PERQ
            // is ignoring video interrupts and letting the display run free...
            _frames = new DirectBitmap[2];
            _frames[0] = new DirectBitmap(size.Width, size.Height, size.Width / 8, PixelFormat.Format1bppIndexed);
            _frames[1] = new DirectBitmap(size.Width, size.Height, size.Width / 8, PixelFormat.Format1bppIndexed);

            _writePtr = 0;

            // Point at the first image while the other is being filled in
            this.Image = _frames[ReadPtr].Bitmap;
        }

        public int ReadPtr
        {
            //get { return (_writePtr == 1 ? 0 : 1); }
            get { return _writePtr; }
        }

        public int WritePtr
        {
            get { return _writePtr; }
            set { _writePtr = value; }
        }

        /// <summary>
        /// Writes a 64-bit (PERQ quad word) to the current bitmap.
        /// </summary>
        public void WriteDisplayQuad(int addr, ulong quad)
        {
            _frames[WritePtr].SetQuad(addr, quad);
        }

        /// <summary>
        /// Reads a quad word from the current bitmap.
        /// </summary>
        public ulong ReadDisplayQuad(int addr)
        {
            return _frames[WritePtr].GetQuad(addr);
        }

        /// <summary>
        /// Render a completed frame and swap the write pointer to start
        /// scribbling in the other one.  (This does an Invalidate behind
        /// the scenes.)
        /// </summary>
        public void RenderBitmap()
        {
            // Assign the just completed bitmap to the picturebox.
            this.Image = _frames[WritePtr].Bitmap;

            // Swap pointers.  This is too clever by half.
            //WritePtr = ReadPtr;
        }

        /// <summary>
        /// Save a screenshot of the read buffer to an image file.
        /// </summary>
        public void Save(string path, ImageCodecInfo codec, EncoderParameters parms)
        {
            _frames[ReadPtr].Bitmap.Save(path, codec, parms);
        }

        DirectBitmap[] _frames;
        private int _writePtr;
    }
}
