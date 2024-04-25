rosshutdown
masterhostIP = '192.168.160.128';
rosinit(masterhostIP)
[ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

% collision_objects = cell(numel(model.ModelNames),1)
models = getModels;  

for i = 12:1:numel(models.ModelNames) 
    % 12:1:39
    model_name = models.ModelNames{i};
    fprintf('Currently working on %s\n', model_name)

    disp(class(model_name));

    % Convert model_name to an integer
    % model_num = str2double(model_name);

    goHome('qr')
    resetWorld

    % pouches = [12, 13, 14, 15, 16, 17, 18, 19];
    % cans = [20, 22, 23, 24, 25, 26, 27, 29];
    % bottles_cans_laying = [31, 32, 36, 38, 39];
    % bottles = [33, 34, 35, 37];

    pouches = {'pouch1', 'pouch2', 'pouch3', 'pouch3', 'pouch4', 'pouch5', 'pouch6', 'pouch7','pouch8'};
    cans = {'gCan1', 'gCan3', 'gCan4', 'rCan1', 'rCan2', 'rCan3', 'yCan1', 'yCan3'};
    bottles_cans_laying = {'rBottle1', 'rBottle2', 'yBottle1', 'yBottle3', 'yBottle4'};
    bottles = {'bBottle1', 'bBottle2', 'bBottle3', 'yBottle2'};

    if any(strcmp(model_name, pouches))
        callPouch(model_name, env, ur5e, config);
        
    elseif any(strcmp(model_name, cans))
        callCan(model_name, env, ur5e, config);
        
    elseif any(strcmp(model_name, bottles_cans_laying))
        callObjectLaying(model_name, env, ur5e, config);
        
    elseif any(strcmp(model_name, bottles))
        callBottle(model_name, env, ur5e, config);
    end
    
end

% Objects laying down:
% rBottle2, rBottle1, yBottle1, yBottle4, bBottle3
% gCan2, yCan4, yCan2




    


