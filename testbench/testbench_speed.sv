module testbench (
    input clk,
    input  reset,
    output signed [31:0] readdata,
	input hall_sensor1,
	input hall_sensor2,
	input hall_sensor3
  );

	RikshawControl DUT(
		clock, reset, hall_sensor1, hall_sensor2, hall_sensor3, readdata
	);

 
endmodule