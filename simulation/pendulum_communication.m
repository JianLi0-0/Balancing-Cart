function pendulum_communication(block)

setup(block);

end

function setup(block)

block.NumDialogPrms = 2;%输入参数数目为1
block.NumOutputPorts = 4;%输出变量个数
block.NumInputPorts  = 1;
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
for i=1:4
    block.OutputPort(i).Dimensions = [1 1]; %[block.DialogPrm(1).Data 1];
    block.OutputPort(i).DatatypeID = 0;% double
    block.OutputPort(i).Complexity = 'Real';
    block.OutputPort(i).SamplingMode = 'Sample';
end
block.InputPort(1).Dimensions = [4 1]; %[block.DialogPrm(1).Data 1];
block.InputPort(1).DatatypeID = 1;% double
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).SamplingMode = 'Sample';
sampletimes = block.DialogPrm(2).Data;
block.SampleTimes = [sampletimes 0];


block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
end

function Start(block)
delete(instrfindall)
global obj;

com_num = block.DialogPrm(1).Data;%串口号
com_serial=['COM',num2str(com_num)];
baud=115200;

obj=serial(com_serial,'baudrate',baud,'databits',8,'stopbits',1);
obj.InputBufferSize = 32; %缓冲区
obj.OutputBufferSize = 8;
fopen(obj);

end

function Outputs(block)

global obj;




%send current to stm32
u = block.InputPort(1).Data;

fwrite(obj,10,'uchar');%data head 0x0A
fwrite(obj,13,'uchar');%data head 0x0D
fwrite(obj,u(1),'float32');
fwrite(obj,u(2),'float32');
fwrite(obj,u(3),'float32');
fwrite(obj,u(4),'float32');
fwrite(obj,13,'uchar');%data head 0x0D
fwrite(obj,10,'uchar');%data head 0x0A

%receive state variables from stm32
% % head=fread(obj,1,'uint8');%int16 double
% % data=fread(obj,4,'float32');%int16 double
% % tail=fread(obj,1,'uint8');%int16 double
% % x_dot = 0;
% % theta_dot = 0;
% % x = 0;
% % theta = 0;
% % if head == 10
% %     if tail == 13
% %         x = data(1);
% %         theta = data(2);
% %         x_dot = data(3);
% %         theta_dot = data(4);
% %     end
% % end
block.OutputPort(1).Data = 0;%base velocity
block.OutputPort(2).Data = 0;%body velocity
block.OutputPort(3).Data = 0;%base coordinate
block.OutputPort(4).Data = 0;%body angle


end
