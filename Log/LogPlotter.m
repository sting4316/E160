% Plot E160 Simulation Data %

clf;

date = '18-04-23';
time = '10.40.38';

fileBot0 = char(strcat('Bot0_', date,{' '}, time, '.txt'));
fileBot1 = char(strcat('Bot1_', date,{' '}, time, '.txt'));
fileBot2 = char(strcat('Bot2_', date,{' '}, time, '.txt'));
delimiterIn = ' ';
headerlinesIn = 1;
A0 = importdata(fileBot0,delimiterIn,headerlinesIn);
A1 = importdata(fileBot1,delimiterIn,headerlinesIn);
A2 = importdata(fileBot2,delimiterIn,headerlinesIn);

StateBot0 = A0.data(:, 1:2);
StateBot1 = A1.data(:, 1:2);
StateBot2 = A2.data(:, 1:2);

NodeDesBot0 = A0.data(:, 3);
NodeDesBot1 = A1.data(:, 3);
NodeDesBot2 = A2.data(:, 3);

NodeDes = [NodeDesBot0; NodeDesBot1; NodeDesBot2];
percentDist = getDistribution(NodeDes);
nodes = 1:6;

BatteryBot0 = A0.data(:, 4);
BatteryBot1 = A1.data(:, 4);
BatteryBot2 = A2.data(:, 4);

figure(1)
plot(StateBot0(:, 1), StateBot0(:, 2), '.')
hold on
plot(StateBot1(:, 1), StateBot1(:, 2), '.')
plot(StateBot2(:, 1), StateBot2(:, 2), '.')
xlabel('X position')
ylabel('Y position')

figure(2)

stem(nodes, percentDist)
xlim([0, 7])
ylim([0, 1])
xlabel('Node')
ylabel('Distribution')

figure(3)
plot(BatteryBot0)
hold on
plot(BatteryBot1)
plot(BatteryBot2)
ylim([0 100]);
xlabel('Time (s)')
ylabel('Battery Percentage')