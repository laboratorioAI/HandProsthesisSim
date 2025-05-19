% Load the BufferManager .NET assembly
asmPath = 'D:\Repositorios_Politecnica\TesisProj\BufferManager\obj\Debug\BufferManager.dll';  % change this to your DLL path
NET.addAssembly(asmPath);

% Create an instance of BufferManager
bm = BufferPrint.BufferManager();
bm.CreateOrOpenSharedMemory();





% Define the list of actions to push
actions = {
    'writte', 'RPkMdJ', single(0.00);
    'read__', 'RPkMdJ', single(0.00);
};

% Push each entry
for i = 1:size(actions, 1)
    action  = string(actions{i,1});
    jointId = string(actions{i,2});
    angle   = actions{i,3};

    msg = bm.WriteToBuffer(action, jointId, angle);

    % Print feedback
    disp(char(msg.ToString()));
end

% Cleanup
bm.CloseSharedMemory();
