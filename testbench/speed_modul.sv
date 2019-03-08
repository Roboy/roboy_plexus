module RikshawControl (
	input clock,
	input reset,
	// this is for the avalon interface
	input hall_sensor1,
	input hall_sensor2,
	input hall_sensor3,
	output [31:0] readdata,
	output [31:0] counterData,
	output divclk
);

reg [31:0] returnvalue;
assign readdata = returnvalue;


//clk is 50000000  divid it o 500
wire clk_out;
wire [31:0] speed_cnt;

assign counterData = speed_cnt;
assign divclk = clk_out;

reg hall_reset = 0;
reg [31:0] speed;	
 
clock_divider cd1(clock, reset, clk_out);
speed_sub ss1(clk_out,reset,hall_reset, speed_cnt);

always @(posedge hall_sensor1, posedge hall_sensor2, posedge hall_sensor3) begin: SPEED_CALC
	speed = speed_cnt;
	for (int i = 0; i < 2; i = i +1) begin
		wait(!clk_out); 
		wait(clk_out);
	end
	hall_reset = 1;
	wait(!clk_out); 
	wait(clk_out);
	hall_reset = 0;
end


// the following iterface handles read requests via lightweight axi bridge
// the upper 8 bit of the read address define which value we want to read
// the lower 8 bit of the read address define for which motor
always @(posedge clock, posedge reset) begin: AVALON_READ_INTERFACE
	returnvalue <= speed;
end

endmodule

/*module speed_sub (input clock, input reset, output [31:0] cnt);	
	always @(posedge clock, posedge reset) begin 
		if (reset) begin
			cnt <= 32'd0 ;
		end else begin
			cnt <= cnt + 32'd1;
			if (cnt > 7000) begin
				cnt <= 32'd0 ;
			end
		end
		
	end
endmodule*/



 module speed_sub(
	input clk, 
	input reset, 
	input reset_hall,
	output[32:0] counter
 );
reg [32:0] counter_up;

always @(posedge clk or posedge reset or posedge reset_hall) begin
		if(reset)
			counter_up <= 32'd0;
		else if(reset_hall)
			counter_up <= 32'd0;
		else if(counter_up > 70000)
			counter_up <= 32'd0;
		else 
			counter_up <= counter_up + 32'd1;
	end 
	assign counter = counter_up;
endmodule


module clock_divider ( 
	input clock,
	input reset,
	output out_clk 
);
	reg out_clk_reg;
	reg [32:0] counter_up;

	always @(posedge clock)
		if (reset) begin
			  out_clk_reg <= 1'b0;
			  counter_up <= 32'd0;
		end else begin
			counter_up <= counter_up + 32'd1;
			if(counter_up == 4) begin
				counter_up <= 32'd0;
				out_clk_reg <= ~out_clk_reg;
			end
		end
	assign out_clk = out_clk_reg;
endmodule


module RikshawControl_tb ();
	reg clock, reset, hall_sensor1, hall_sensor2, hall_sensor3;
	// this is for the avalon interface
	wire [31:0] readdata;
	wire [31:0] counterData;
	wire div_clk;

parameter stimDelay = 10;
RikshawControl DUT(clock, reset, hall_sensor1, hall_sensor2, hall_sensor3, readdata, counterData, div_clk);
initial
begin
 clock = 0; reset = 1; hall_sensor1 = 0; hall_sensor2 = 0; hall_sensor3 = 0;
 
 #(stimDelay) reset = 1;clock = 1;
 #(stimDelay) reset = 0;clock = 0; 
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0; 
 #(stimDelay) clock = 1; 
 #(stimDelay) clock = 0; 
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0; hall_sensor1=1;
 #(stimDelay) clock = 1; hall_sensor1=0;
 #(stimDelay) clock = 0; hall_sensor1=1;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;hall_sensor1=1;
 #(stimDelay) clock = 1;hall_sensor1=1;
 #(stimDelay) clock = 0;hall_sensor1=0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0; 
 #(stimDelay) clock = 1; 
 #(stimDelay) clock = 0; 
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0; 
 #(stimDelay) clock = 1; 
 #(stimDelay) clock = 0; 
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0; 
 #(stimDelay) clock = 1; 
 #(stimDelay) clock = 0; 
 #(stimDelay) clock = 1;
 #(stimDelay) clock = 0;
#100; //Let simulation finish
end
endmodule