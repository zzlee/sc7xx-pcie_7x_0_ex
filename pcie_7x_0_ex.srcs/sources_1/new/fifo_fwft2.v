module fifo_fwft2 #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 16
)(
    input  wire                   clk,
    input  wire                   rst_n,
    // 寫入端口
    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  wr_data,
    output wire                   full,
    // 讀出端口
    input  wire                   rd_en,
    output wire [DATA_WIDTH-1:0]  rd_data,
    output wire                   empty,
    // 回報目前FIFO紀錄筆數
    output wire [$clog2(DEPTH+1)-1:0] size
);
    // 記憶體陣列
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [$clog2(DEPTH)-1:0] wr_ptr, rd_ptr;
    reg [$clog2(DEPTH+1)-1:0] count; // 記錄元素數量

    // FWFT數據輸出
    assign rd_data = mem[rd_ptr];

    // 滿/空及size判斷
    assign full  = (count == DEPTH);
    assign empty = (count == 0);
    assign size  = count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
        end else begin
            case ({wr_en, rd_en})
                2'b10: if (!full) begin // 寫入
                    mem[wr_ptr] <= wr_data;
                    wr_ptr <= (wr_ptr + 1) % DEPTH;
                    count  <= count + 1;
                end
                2'b01: if (!empty) begin // 讀出
                    rd_ptr <= (rd_ptr + 1) % DEPTH;
                    count  <= count - 1;
                end
                2'b11: if (!empty && !full) begin // 寫讀同時
                    mem[wr_ptr] <= wr_data;
                    wr_ptr <= (wr_ptr + 1) % DEPTH;
                    rd_ptr <= (rd_ptr + 1) % DEPTH;
                    // count不變
                end
                default: ; // 無任何操作
            endcase
        end
    end
endmodule
