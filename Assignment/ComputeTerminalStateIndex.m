function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state in the
%stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix

global DROP_OFF
[m_DO, n_DO] = find(map == DROP_OFF);
stateIndex = find(stateSpace(:,1) == m_DO & stateSpace(:,2) == n_DO & stateSpace(:,3) == 1);
                  
end
