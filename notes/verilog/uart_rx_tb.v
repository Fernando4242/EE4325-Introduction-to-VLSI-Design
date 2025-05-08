// Testbench for uart_rx
`timescale 1ns/1ns 
module uart_rx_tb;

// Variables
reg        clock;
reg        reset;
reg        rx_i;
wire [7:0] rx_data;
wire       rx_data_valid;
wire       rx_error;
  
// Instantiate design under test
uart_rx DUT(.clock(clock),.reset(reset),.rx_i(rx_i),.rx_data(rx_data),
	    .rx_data_valid(rx_data_valid),.rx_error(rx_error));
          
// Generate the clock
initial begin
  clock = 1'b0;
  forever #5 clock = ~clock;
end

// Stimulus + Checking
initial begin
  $display("Reset");
  reset = 1; 
  rx_i = 1'bx;
  display;
    
  $display("Remove reset");
  reset = 0;
  rx_i = 1'b1;
  display;

  $display("Idle");  rx_i = 1'b1; display;

  // Valid data sent, random data
  $display("Start"); rx_i = 1'b0; display; // Start bit
  $display("Bit 0"); rx_i = 1'b1; display; // LSB
  $display("Bit 1"); rx_i = 1'b0; display;
  $display("Bit 2"); rx_i = 1'b1; display;
  $display("Bit 3"); rx_i = 1'b0; display;
  $display("Bit 4"); rx_i = 1'b1; display;
  $display("Bit 5"); rx_i = 1'b1; display;
  $display("Bit 6"); rx_i = 1'b1; display;
  $display("Bit 7"); rx_i = 1'b0; display; // MSB
  $display("Stop");  rx_i = 1'b1; display; // Valid stop bit

  // Valid data sent, all 1's
  $display("Start"); rx_i = 1'b0; display; // Start bit
  $display("Bit 0"); rx_i = 1'b1; display; // LSB
  $display("Bit 1"); rx_i = 1'b1; display;
  $display("Bit 2"); rx_i = 1'b1; display;
  $display("Bit 3"); rx_i = 1'b1; display;
  $display("Bit 4"); rx_i = 1'b1; display;
  $display("Bit 5"); rx_i = 1'b1; display;
  $display("Bit 6"); rx_i = 1'b1; display;
  $display("Bit 7"); rx_i = 1'b1; display; // MSB
  $display("Stop");  rx_i = 1'b1; display; // Valid stop bit

  // Longer idle state
  $display("Idle");  rx_i = 1'b1; display;
  $display("Idle");  rx_i = 1'b1; display;

  // Valid data sent, all 0's
  $display("Start"); rx_i = 1'b0; display; // Start bit
  $display("Bit 0"); rx_i = 1'b0; display; // LSB
  $display("Bit 1"); rx_i = 1'b0; display;
  $display("Bit 2"); rx_i = 1'b0; display;
  $display("Bit 3"); rx_i = 1'b0; display;
  $display("Bit 4"); rx_i = 1'b0; display;
  $display("Bit 5"); rx_i = 1'b0; display;
  $display("Bit 6"); rx_i = 1'b0; display;
  $display("Bit 7"); rx_i = 1'b0; display; // MSB
  $display("Stop");  rx_i = 1'b1; display; // Valid stop bit

  // Invalid data sent
  $display("Start"); rx_i = 1'b0; display; // Start bit
  $display("Bit 0"); rx_i = 1'b1; display; // LSB
  $display("Bit 1"); rx_i = 1'b0; display;
  $display("Bit 2"); rx_i = 1'b1; display;
  $display("Bit 3"); rx_i = 1'b1; display;
  $display("Bit 4"); rx_i = 1'b1; display;
  $display("Bit 5"); rx_i = 1'b1; display;
  $display("Bit 6"); rx_i = 1'b1; display;
  $display("Bit 7"); rx_i = 1'b1; display; // MSB
  $display("!Stop"); rx_i = 1'b0; display; // Invalid stop bit

  // Don't leave error until 1 is sent
  $display("Error"); rx_i = 1'b0; display;
  $display("Idle");  rx_i = 1'b1; display;
end
  
task display;
  begin 
    #10;
    $display("reset:%0h, rx_i:%0h, rx_data:%0h, rx_data_valid:%0h, rx_error:%0h",
	      reset, rx_i, rx_data, rx_data_valid, rx_error);
  end
endtask

endmodule // End of module
