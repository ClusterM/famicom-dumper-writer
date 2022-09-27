module FamicomDumper # (
  parameter LEDS_TIMER_SIZE = 12,
  parameter VERSION_3 = 1
)
(
   input m2,
   input master_clock,
   input ne1,
   input ne2,
   input nwe,
   input noe,
   input a13,
   input a15, 
   output nwait,
 
   output romsel,
   output cpu_rw,
   output ppu_rd,
   output ppu_wr,
   output na13,
   output cpu_dir,
   output cpu_oe,
   output ppu_dir,
   output ppu_oe,
 
   input coolboy_mode,
   output coolboy_oe,
   output coolboy_we,
 
   output led_prg_read,
   output led_prg_write,
   output led_chr_read,
   output led_chr_write
);

assign romsel = !(m2 && a15 && ne1_active);
assign cpu_rw = reg_cpu_rw /*|| (coolboy_mode && VERSION_3)*/;
assign cpu_oe = !cpu_shifter_enabled;
assign cpu_dir = !reg_cpu_rw;
assign ppu_rd = !(!ne2 && !noe);
assign ppu_wr = !(!ne2 && !nwe);
assign ppu_oe = !(!ne2 && ne1);
assign ppu_dir = !(!ne2 && !noe);
assign na13 = !a13;
assign nwait = !waiting;
assign coolboy_oe = !(ne1_active && m2 && a15 && reg_cpu_rw);
assign coolboy_we = !(ne1_active && m2 && a15 && !reg_cpu_rw);

reg [2:0] stage = 0;
reg [5:0] wait_timer = 0;
reg [4:0] neg_m2_timer = 0;
wire waiting = wait_timer < (nwe ? 3'b111 : 4'b1111);
reg cpu_shifter_enabled = 0;
reg reg_cpu_rw = 1;

wire ne1_active = !ne1 && (!noe || !nwe);

reg [1:0] active_led = 0;
reg [LEDS_TIMER_SIZE:0] led_timer = 0;
assign led_prg_read = (active_led == 2'b00) && led_on;
assign led_prg_write = (active_led == 2'b01) && led_on;
assign led_chr_read = (active_led == 2'b10) && led_on;
assign led_chr_write = (active_led == 2'b11) && led_on;
wire led_on = led_timer < ((1 << (LEDS_TIMER_SIZE + 1)) - 1);

always @ (negedge master_clock)
begin
   if (m2)
   begin
      neg_m2_timer = 0;
   end else begin
      neg_m2_timer = neg_m2_timer + 1'b1;
   end
   
   // waiting for ne1
   if (!ne1_active) begin   
      stage = (!m2 && neg_m2_timer < 7) ? 2 : 0;
      wait_timer = 0;
      cpu_shifter_enabled = 0;
      reg_cpu_rw = 1; // read mode
   end 
   else if (stage == 0) // low M2?
   begin
      // waiting for high M2
      if (m2) stage = 1;
   end 
   else if (stage == 1) // low M2
   begin
      // waiting for low M2
      if (!m2) stage = 2;
   end 
   else if (stage == 2) // low M2
   begin
      // set direction to writing if need
      if (!nwe) reg_cpu_rw = 0;       
      // enable shifter
      cpu_shifter_enabled = 1;
      // waiting for high M2
      // actual reading/writing starts here
      if (m2) stage = 3;
   end 
   else if (stage == 3) // high M2
   begin
      if (ne1_active && waiting)
      begin
         wait_timer = wait_timer + 1'b1;
      end
   end   
end

always @ (posedge m2)
begin
  if (led_on) led_timer = led_timer + 1; // leds timer
  
  if (!ne1 && !noe) 
  begin
    active_led = 2'b00;
    led_timer = 0;
  end

  if (!ne1 && !nwe)
  begin
    active_led = 2'b01;
    led_timer = 0;
  end  

  if (!ne2 && !noe) 
  begin
    active_led = 2'b10;
    led_timer = 0;
  end

  if (!ne2 && !nwe)
  begin
    active_led = 2'b11;
    led_timer = 0;
  end  
end

endmodule