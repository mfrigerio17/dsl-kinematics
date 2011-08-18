scale = 10000;
s = 0;
stateScaling = 10;
for i = 1 : 100
    q   = [rand() rand()] * stateScaling;
    qd  = [rand() rand()] * stateScaling;
    qdd = [rand() rand()] * stateScaling;
    % Calls my function for the inverse dynamics of a two link planar arm
    idm = uint32(id_planar2(q,qd,qdd) * scale);
    % This call Featherstone's implementation. planar2 must be an equivalent
    %  model
    idf = uint32(ID(planar2, q,qd,qdd) *scale);
    
    s = s + sum(idf ~= idm);
    if any(idf ~= idm)
    	[idf idm]
    end
end

