module vram_a (
	input logic clock,
	input logic wren_a,
	input logic data_a,
	input logic byteena_a,
    input logic [13:0] address_a,
    output logic [31:0] q_a,
	
	input logic wren_b,
	input logic data_b,
    input logic [13:0] address_b,
    output logic [31:0] q_b
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
	if (wren_a) mem[address_a] <= data_a;
	
	if (wren_b) mem[address_b] <= data_b;
	
	//$display("data=%h addr=%h word=%h",data, addr, addr[31:2]);
end

assign q_a= mem[address_a];
assign q_b= mem[address_b];

endmodule: vram_a
