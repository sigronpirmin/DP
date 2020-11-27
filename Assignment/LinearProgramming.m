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
G(TERMINAL_STATE_INDEX,:) = [];
P(TERMINAL_STATE_INDEX,:,:) = [];
P(:,TERMINAL_STATE_INDEX,:) = [];


% initialize optimal cost
J_opt = zeros(K,1);
J_togo = zeros(K-1,1);

% initialize optimal policy
u_opt_ind = ones(K,1);
u_pol = ones(K-1,1);

f = ones(K-1,1) * -1;

A = [];
b = [];
for in = 1:5
    A = [A; eye(K-1) - P(:,:,in)];
    b = [b; G(:,in)];
end
    
% Remove transitions with infinite cost
A(b==Inf,:) = [];
b(b==Inf) = [];

J_togo = linprog(f,A,b');

% Policy improvement for optimal policy
for j = 1:K-1
    [~, u_pol(j)] = min(G(j,:)' + squeeze(P(j,:,:))'*J_togo(:));
end
    
    

J_opt = [J_togo(1:TERMINAL_STATE_INDEX-1);0;J_togo(TERMINAL_STATE_INDEX:end)];
u_opt_ind = [u_pol(1:TERMINAL_STATE_INDEX-1);HOVER;u_pol(TERMINAL_STATE_INDEX:end)];

end

