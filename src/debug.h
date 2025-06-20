//
// MIT License
//
// Copyright (c) 2020 Stuart Wilkins
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef SRC_DEBUG_H_
#define SRC_DEBUG_H_

#include <SEGGER_RTT.h>

#ifdef DEBUG_OUTPUT
  #define DEBUG_PRINT(fmt, ...) \
    SEGGER_RTT_printf(0, "%s:%d:%s(): " fmt, \
            __FILE__, __LINE__, __func__, __VA_ARGS__);
  #define DEBUG_COMMENT(fmt) \
    SEGGER_RTT_printf(0, "%s:%d:%s(): " fmt, \
            __FILE__, __LINE__, __func__);
#else
  #define DEBUG_PRINT(fmt, ...) \
      do {} while (0)
  #define DEBUG_COMMENT(fmt) \
      do {} while (0)
#endif

#endif  // SRC_DEBUG_H_
