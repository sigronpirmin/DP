function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
G(TERMINAL_STATE_INDEX,:) = zeros(5,1);

% initialize optimal cost
J_opt = zeros(K,1);

% initialize optimal policy
u_opt_ind = ones(K,1)*5;
u_pol = ones(K,1)*5;

% helper matrices
P_inter = zeros(K);
G_inter = zeros(K,1);

iter = 0;
while(1)
    iter = iter + 1;
    for i  = 1:K
        P_inter(i,:) = P(i,:,u_opt_ind(i));
        G_inter(i) = G(i,u_opt_ind(i));
    end
    
    J_h = (eye(K) - P_inter) \ G_inter;
    % update cost
    for k = 1:K
        if k ~= TERMINAL_STATE_INDEX
            [J_opt(k), u_pol(k)] = min(G(k,:)' + squeeze(P(k,:,:))'*J_h(:));
        end
    end
    if sum(u_opt_ind ~= u_pol)
        u_opt_ind = u_pol;
    else
        u_opt_ind = u_pol;
        break
    end


end
