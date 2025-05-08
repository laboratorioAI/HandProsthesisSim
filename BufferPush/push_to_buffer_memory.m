% push data information to shared buffer with Matlab
% load library
NET.addAssembly('D:\Repositorios_Politecnica\TesisProj\BufferManager\obj\Debug\BufferManager.dll');
% instanciate object
bm = BufferPrint.BufferManager();
bm.CreateOrOpenSharedMemory();
% push to memory
msg = bm.WriteToBuffer(single(3.14));
disp(char(msg.ToString()));
% close buffer memory
bm.CloseSharedMemory();
% recuerda que para ver el valor impreso en la consola hay que
% ejecutar el proyecto C# llamado Buffer print, a pesar que la
% libreria a la que se referencia esta en el proyecto C# llamado
% Buffer manager