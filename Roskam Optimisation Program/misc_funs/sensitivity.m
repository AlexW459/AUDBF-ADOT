% Sensitivity analysis
% Done by: Muhammad Rabay (a1810169)

clear all;
clc;

%% Set up

% Scoring parameters
W_p = 11; % [lbs], payload weight in pounds in M2
t_3l = 126; % [s], time to complete 3 laps in M2
M2N = W_p/t_3l; % personalised score
M2max = M2N*1.5; % max score for M2

M2N/M2max

N_L = 6; %   [-], number of laps in 5 minutes M3
B_b_score = 2.5; % [-], glider landing position from ejection, also called bonus box score in M3
W_X1 = 0.22; % [lbs], payload weight of glider in M3
M3N = N_L + (B_b_score/W_X1);
M3max = M3N*1.5; % assuming that our baseline would land us in ~67th percentile
change = linspace(-100,100,101); % labelling the change

%% Calculating the score based on the sensitivity

% varying payload weight
S_T_p = 4 + ((W_p.*(1+change./100))./t_3l)./(M2max) + M3N/M3max;

% varying time for 3 laps
S_T_t = 4 + ((W_p)./(t_3l.*(1+change./100)))./(M2max) + M3N/M3max;

% varying number of laps
S_T_N = 4 + M2N/M2max + (round(N_L.*(1+change./100)) + B_b_score/W_X1)./M3max;

% varying bonus box score
S_T_B = 4 + M2N/M2max+ ((N_L) + round(B_b_score.*(1+change./100))./W_X1)./M3max;

% varying glider weight
S_T_X1 = 4 + M2N/M2max+ ((N_L) + (B_b_score)./(W_X1.*(1+change./100)))./M3max;


%% plotting the sensitivity
figure()
linewidth = 2;
plot(change,S_T_p,':r','LineWidth',linewidth,'DisplayName','Payload Weight,M2');
hold on;
plot(change,S_T_t,'--c','LineWidth',linewidth,'DisplayName','Time for 3 laps,M2');
hold on
plot(change,S_T_N,'-b','LineWidth',linewidth,'DisplayName','NO. of laps, M3');
hold on;
plot(change,S_T_B,'-g','LineWidth',linewidth,'DisplayName','Bonus box score,M3');
hold on;
plot(change,S_T_X1,'-k','LineWidth',linewidth,'DisplayName','X-1 Weight,M3');
xlabel('Change in score [%]');
ylabel('Score');
ylim([4 6]);
legend;
hold on;


%% individual sensitvity mission 2 time scoring on the final score

T_3L = linspace(90,150,7);

% varying time for 3 laps
for i = 1:length(T_3L)
    time = (T_3L(i).*(1+change/100));
    S_M2_time = 1 + (W_p./time)./M2max;
    S_M3_time = 2 + M3N./M3max;
    S_T_time(i,:) = S_M2_time + S_M3_time;
end

for i = 1:length(T_3L)
delta_score_time(i,:) = S_T_time(i,:) - S_T_time(i,50);
end

figure()
hold on; box on; grid on;
for i = 1:length(T_3L)
plot(change,S_T_time(i,:) - S_T_time(i,50),'LineWidth',2,'DisplayName',['Time for 3 laps: ',num2str(T_3L(i)),' s.']);
hold on;
end
xlim([-50 50]);
ylim([-0.5 1]);
legend
title('Inidividual sensitivity on final score: time required for 3 laps [M2]')



%% individual sensitvity mission 2 time scoring on the final score

W_P = linspace(0.1,0.55,7);

% varying time for 3 laps
for i = 1:length(W_P)
    payload = (W_P(i).*(1+change/100));
    S_T_payload(i,:) = 1 + (payload./t_3l)./M2max + 2 + M3N./M3max;
end


figure()
hold on; box on; grid on;
for i = 1:length(T_3L)
plot(change,S_T_payload(i,:) - S_T_payload(i,50),'LineWidth',2,'DisplayName',['Payload weight: ',num2str(W_P(i)),' lbs.']);
hold on;
end
xlim([-50 50]);
ylim([-0.5 0.5]);
legend
title('Inidividual sensitivity on final score: Payload weight [M2]')
