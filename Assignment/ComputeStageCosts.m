function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    G = zeros(K,5);
    % find residents
    [m_res, n_res] = find(map == SHOOTER);
    % find base index in stateSpace
    [m_base, n_base] = find(map == BASE);
    baseIndex = find(stateSpace(:,1) == m_base & stateSpace(:,2) == n_base & stateSpace(:,3) == 0);
    
    % dimensions of map
    m = length(map(:,1));
    n = length(map(1,:));
    
    % change in state for each input
    chIn = [0 0 1 -1 0;1 -1 0 0 0];         % first column change in m, second change in n
    
    % iterate over all admissible inputs
    for in = 1:5
        % iterate over all admissible states
        for state = 1:K
            m_des = stateSpace(state,1)+chIn(1,in);
            n_des = stateSpace(state,2)+chIn(2,in);
            if (1 <= m_des &  m_des <= m) & (1 <= n_des & n_des <= n)
                if map(m_des,n_des) == TREE
                    G(state,in) = inf;
                else
                    % probability of crashing
                    P_crash = 0;
                    % due to wind and shooting
                    for i = 1:4
                        if (m_des+chIn(1,i) == 0) | (m_des+chIn(1,i) > m) | (n_des+chIn(2,i) == 0) | (n_des+chIn(2,i) > n)
                            P_crash = P_crash + P_WIND/4;
                        else
                            if map(m_des+chIn(1,i),n_des+chIn(2,i)) == TREE
                                P_crash = P_crash + P_WIND/4;
                            end
                            %compute distance to shooters
                            dist = abs(m_res - (m_des+chIn(1,i))) + abs(n_res - (n_des+chIn(2,i)));   
                            for res = 1:length(dist)
                                if dist(res) <= R
                                    P_crash = P_crash + P_WIND/4 * GAMMA / (dist(res)+1);
                                end
                            end 
                        end
                    end
                    % due to shooting at x_des
                    %%% EDIT: Here no additional wind shift for distance?
                    %dist = abs(m_res - m_des + abs(n_res - n_des); 
                    %%%
                    dist = abs(m_res - (m_des+chIn(1,i))) + abs(n_res - (n_des+chIn(2,i)));   
                    for res = 1:length(dist)
                        if dist(res) <= R
                            P_crash = P_crash + (1 - P_WIND) * GAMMA / (dist(res)+1);
                        end
                    end
            
                    G(state,in) = 1 * (1-P_crash) + Nc * P_crash;
                end
            else
                    G(state,in) = inf;
            end
        end
    end
end

