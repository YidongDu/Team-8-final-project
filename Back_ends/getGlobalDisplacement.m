function [displacement] = getGlobalDisplacement(measurement,pose)
rel_displacement = pose.R*measurement;
displacement.p = rel_displacement + pose.p;
