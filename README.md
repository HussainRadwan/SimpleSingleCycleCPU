# SimpleSingleCycleCPU
In this project, a single cycle CPU was created using the Verilog Active HDL. The control and the Datapath were included after building the needed components. In each cycle in our circuit, an instruction will be implemented. The cycle time depends on which instruction takes the longest time in implementation due to working in the whole stages: fetch, decode, execute, memory and write back, which is LW instruction. The project's ultimate objective is to make the CPU execute each instruction after storing a particular set of instructions in the instruction memory. 

## Datapath
Here is the data path that build to construct the single cycle CPU:
![Screenshot](https://github.com/user-attachments/assets/56d3ae91-6866-47f4-b173-9c3b9c44dea2)

## Control units
Main control unit and PC control unit. Notice that: ""didnt use the ALU control unit; because it was built implicitly in the Alu module.""
![Screenshot](https://github.com/user-attachments/assets/a29347c9-38ff-402c-9d2a-20cc5236d068)


## Instructions
![Screenshot](https://github.com/user-attachments/assets/9104ae89-5535-4d59-8e38-bc3268420233)


