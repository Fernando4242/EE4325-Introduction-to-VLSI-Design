`timescale 1ns/1ns
//-----------------------------------------------------
// This is a UART Receiver Program
// Design Name : uart_rx
// File Name   : uart_rx.v
//-----------------------------------------------------
module uart_rx (
clock         , // Clock
reset         , // Active high reset
rx_i          , // Serial input
rx_data       , // Received data
rx_data_valid , // Received data is valid
rx_error      , // Error found     
);

//-------------Input Ports----------------------------
input  clock;
input  reset;
input  rx_i;
//-------------Output Ports----------------------------
output [7:0] rx_data;
output reg   rx_data_valid;
output reg   rx_error;
//-------------Internal Constants-------------------------------------------
parameter IDLE = 2'b00, DATA = 2'b01, CHECK_STOP = 2'b10, ERROR = 2'b11;  // Binary encoding states of FSM
//-------------Internal Variables-------------------------------------------
reg [1:0] state      ; // Current state of FSM
reg [1:0] next_state ; // Next state of FSM
reg [2:0] counter    ; // Count number of data bits
reg [7:0] shift	     ; // Received data, shift right

//----------Code starts Here------------------------
always @ (*) begin
  case(state)
    IDLE : if (rx_i == 1'b1) begin  
             next_state = IDLE;
           end else begin
             next_state = DATA;
           end
    DATA : if (counter < 7) begin	// Get 8 bits of data
             next_state = DATA;
           end else begin
             next_state = CHECK_STOP;
           end
    CHECK_STOP : if (rx_i == 1'b1) begin
                   next_state = IDLE;
                 end else begin
                   next_state = ERROR;
                 end
    ERROR : if (rx_i == 1'b1) begin
	      next_state = IDLE;
            end else begin
	      next_state = ERROR;
            end
    default : next_state = IDLE;
  endcase
end

//----------Seq Logic-----------------------------
always @ (posedge clock) begin   
  if (reset == 1'b1) begin
    state <= #1 IDLE;
  end else begin
    state <= #1 next_state;
  end
end

//---------- Logic--------------------------------
always @ (posedge clock) begin   
  if (reset == 1'b1) begin
    counter       <= #1 1'b0;
    shift         <= #1 8'b0;
    rx_error      <= #1 1'b0;
    rx_data_valid <= #1 1'b0;
  end else begin
    case(state)
      IDLE : begin
               counter       <= #1 1'b0;
               shift         <= #1 shift;
	       rx_error      <= #1 1'b0;
               rx_data_valid <= #1 rx_data_valid;
             end
      DATA : begin
               counter       <= #1 counter + 1;	   // Increment counter for every data bit received
	       shift[7]      <= #1 rx_i;
               shift[6:0]    <= #1 shift[7:1];
               rx_error      <= #1 1'b0;
               rx_data_valid <= #1 1'b0;
             end
      CHECK_STOP : begin
                     counter  <= #1 1'b0;
		     shift    <= #1 shift;
                     rx_error <= #1 1'b0;
		     if (rx_i == 1'b1) begin
		       rx_data_valid <= #1 1'b1;
                     end else begin
		       rx_data_valid <= #1 1'b0;
		     end
                   end
      ERROR : begin
                counter       <= #1 1'b0;
		shift         <= #1 shift;
		rx_error      <= #1 1'b1;
                rx_data_valid <= #1 1'b0;
	      end
      default : begin
                  counter       <=  #1  1'b0;
		  shift         <=  #1  8'b0;
		  rx_error      <=  #1  1'b0;
		  rx_data_valid <= #1 1'b0;
                end
    endcase
  end
end

//----------Output Assignment---------------------
assign rx_data = shift;

endmodule // End of module