function [BestSol] = PSO_Tour(model)
CostFunction=@(tour) TourLength(tour,model);
nVar=model.n;
%% ACO Parameters
MaxIt=300;      % Maximum Number of Iterations
nAnt=40;        % Number of Ants (Population Size)
Q=1;
tau0=10*Q/(nVar*mean(model.D(:)));	% Initial Phromone
alpha=1;        % Phromone Exponential Weight
beta=1;         % Heuristic Exponential Weight
rho=0.05;       % Evaporation Rate
%% Initialization
eta=1./model.D;             % Heuristic Information Matrix
tau=tau0*ones(nVar,nVar);   % Phromone Matrix
BestCost=zeros(MaxIt,1);    % Array to Hold Best Cost Values
% Empty Ant
empty_ant.Tour=[];
empty_ant.Cost=[];
% Ant Colony Matrix
ant=repmat(empty_ant,nAnt,1);
% Best Ant
BestSol.Cost=inf;
%% ACO Main Loop
for it=1:MaxIt
    
    % Move Ants
    for k=1:nAnt
        
        ant(k).Tour=randi([1 nVar]);
        
        for l=2:nVar
            
            i=ant(k).Tour(end);
            
            P=tau(i,:).^alpha.*eta(i,:).^beta;
            
            P(ant(k).Tour)=0;
            
            P=P/sum(P);
            
            j=RouletteWheelSelection(P);
            
            ant(k).Tour=[ant(k).Tour j];
            
        end
        
        ant(k).Cost=CostFunction(ant(k).Tour);
        
        if ant(k).Cost<BestSol.Cost
            BestSol=ant(k);
        end
        
    end
    


end

