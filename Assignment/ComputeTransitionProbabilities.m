function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

K = length(stateSpace(:,1));
P = zeros(K,K,5);
% find residents
[m_res, n_res] = find(map == SHOOTER);
% find base index in stateSpace
[m_base, n_base] = find(map == BASE);
baseIndex = find(stateSpace(:,1) == m_base & stateSpace(:,2) == n_base & stateSpace(:,3) == 0);

m = length(map(:,1));
n = length(map(1,:));

% change in state for each input
chIn = [0 0 1 -1 0;1 -1 0 0 0];         % first column change in m, second change in n
%  iterate over all admissible inputs
for in = 1:5
    % iterate over all admissible start states
    for from = 1:K
        % check whether input is admissible
        if (1 <= stateSpace(from,1)+chIn(1,in)) &  (stateSpace(from,1)+chIn(1,in) <= m) & (1 <= stateSpace(from,2)+chIn(2,in)) & (stateSpace(from,2)+chIn(2,in) <= n)
           if (map(stateSpace(from,1)+chIn(1,in),stateSpace(from,2)+chIn(2,in)) ~= TREE)
               % iterate over all admissible end states
               for to = 1:K
                   % if to-state is no pick-up place, only states with same package-state reachable
                   if (stateSpace(from,3) ~= stateSpace(to,3)) & (map(stateSpace(to,1),stateSpace(to,2)) ~= PICK_UP)
                       P(from,to,in) = 0;
                   else
                       % check whether to-state can be reached
                       if (abs(stateSpace(to,1) - (stateSpace(from,1)+chIn(1,in))) <= 1 & stateSpace(from,2)+chIn(2,in) == stateSpace(to,2)) | (stateSpace(from,1)+chIn(1,in) == stateSpace(to,1) & abs(stateSpace(to,2) - (stateSpace(from,2)+chIn(2,in))) <= 1)
                           % compute distance to each resident
                           dist = abs(m_res - stateSpace(to,1)) + abs(n_res - stateSpace(to,2));
                           % distinguish between reachable states
                           if (stateSpace(to,1) == stateSpace(from,1)+chIn(1,in)) & (stateSpace(to,2) == stateSpace(from,2)+chIn(2,in))
                               P(from,to,in) = 1 - P_WIND;
                               for res = 1:length(dist)
                                   if dist(res) <= R
                                       P(from,to,in) = P(from,to,in) * (1 - GAMMA / (dist(res)+1));
                                   end
                               end  
                               % pick-up procedure
                               if (map(stateSpace(to,1),stateSpace(to,2)) == PICK_UP) & (stateSpace(to,3) == 0)
                                   P(from,to+1,in) = P(from,to,in);
                                   P(from,to,in) = 0;
                               end
                           elseif (map(stateSpace(to,1),stateSpace(to,2)) ~= TREE)
                               P(from,to,in) = P_WIND/4;
                               for res = 1:length(dist)
                                   if dist(res) <= R
                                       P(from,to,in) = P(from,to,in) * (1 - GAMMA / (dist(res)+1));
                                   end
                               end 
                               % pick-up procedure
                               if (map(stateSpace(to,1),stateSpace(to,2)) == PICK_UP) & (stateSpace(to,3) == 0)
                                   P(from,to+1,in) = P(from,to,in);
                                   P(from,to,in) = 0;
                               end
                           end
                       else
                           % to-state is not reachable
                           P(from,to,in) = 0;
                       end
                   end
               end
           else
               P(from,:,in) = 0;
           end
        else
           P(from,:,in) = 0;
        end
        % probability of crashing
        P(from,baseIndex,in) = P(from,baseIndex,in) + (1 - sum(P(from,:,in)));
    end
end

end
