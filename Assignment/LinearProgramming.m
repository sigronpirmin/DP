function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the terminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
G(TERMINAL_STATE_INDEX,:) = zeros(5,1);


% initialize optimal cost
J_opt = zeros(K,1);
J_togo = zeros(K,1);

% initialize optimal policy
u_opt_ind = ones(K,1);

% helper functions
u_opt_help = zeros(K,5);
J_help = zeros(K,5);
iter = 0;

while(1)
    
    iter = iter + 1;
    
    for in = 1:5
        A = G(:,in) + sum(P(:,:,in).*J_togo);
        b = J_opt;
        f = J_opt;
        [u_opt_help(:,in), J_help(:,in)] = linprog(f,A,b');
    end
    [J_opt, u_opt_ind] = max(J_help,[],2);
    

        
    
end
    
    



end

