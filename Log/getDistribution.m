function [percentDistribution] = getDistribution(distribution)

totalTime = length(distribution);

percentDistribution = zeros(6, 1);
for node=1:6
    nodeTime = length(find(distribution==node));
    nodePercentage = nodeTime/totalTime;
    percentDistribution(node, 1) = nodePercentage;
end
end

