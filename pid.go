package main

/*
#include <stdlib.h>
#include "./pid.h"

*/
import "C"
import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"time"
)

func main() {
	pid := C.PIDStruct{
		p:                        0.5,
		i:                        0.1,
		d:                        0.0,
		setpoint:                 50,
		dt:                       1,
		input:                    0,
		lower_limit:              -50,
		upper_limit:              50,
		sum_clamping_coefficient: 1.1,
	}

	C.PID_reset(&pid)

	go func() {
		for {
			val := C.PID_calc(&pid)
			fmt.Printf("P: %.2f I: %.2f D: %.2f SP: %.2f CURR: %.2f VAL: %.2f ERR: %.2f SUM: %.2f\n",
				pid.p, pid.i, pid.d, pid.setpoint, pid.input, val, pid.last_error, pid.sum_error)
			time.Sleep(time.Second)
		}
	}()

	scanner := bufio.NewScanner(os.Stdin)
	for scanner.Scan() {
		if s, err := strconv.ParseFloat(scanner.Text(), 32); err == nil {
			pid.input = C.float(s)
		}
	}

}
