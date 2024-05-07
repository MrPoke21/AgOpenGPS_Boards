/*
 *    CyclicTimer.cpp - A simple class for periodic events from loop()
 *    Copyright (c) 2019 Kees Moerman
 * 
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 * 
 *    This permission notice shall be included in all copies or 
 *    substantial portions of the Software.
 * 
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef CyclicTimer_h
#define CyclicTimer_h

class CyclicTimer {
  private:
    uint16_t period;                    // period in miliseconds
    uint32_t lastTick;                  // last time stamp from millis()

  public:
        /* 
         * Initialise a new timer (without setting the time)
         */
    CyclicTimer(void);
    
        /* 
         * Set the timer period in ms and start running. Period max 65 seconds.
         * Params :
         * uint16_t	Timer period in ms
         */
    void setPeriod(uint16_t newPeriod);
    
        /* 
         * Resets the timer, useful for timeout counters etc
         */
    void reset(void);
    
        /* 
         * Progresses the timer and test/reload if expired
         * Returns :
         * bool	if true, the timer has expired (and is reloaded)
         */
    bool tickAndTest(void);
};


#endif	// CyclicTimer_h


