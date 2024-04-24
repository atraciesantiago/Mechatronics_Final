rosshutdown
masterhostIP = '192.168.160.128';
rosinit(masterhostIP)
[ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT1;

% collision_objects = cell(numel(model.ModelNames),1)

for i = 7:numel(models.ModelNames) 
    model_name = models.ModelNames{i};
    disp('Currently working on %d', model_name)
    Object = callObject(model_name, env);
end





    


