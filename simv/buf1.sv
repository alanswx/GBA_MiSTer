module buf1 (
	input logic clock,
	input logic wren,
	input logic data,
    input logic [13:0] address,
    output logic [31:0] q
	
);
    logic [31:0] mem [16383:0];
	
	/*
    initial begin
        $readmemh("vram_a_sim.txt", vram_A);
    end
	*/
always_ff @(posedge clock)
/*
if (rsta) begin

end
else*/ begin
	if (wren) mem[address] <= data;
	
	
	//$display("data=%h addr=%h word=%h",data, addr, addr[31:2]);
end

assign q= mem[address];

endmodule: buf1
