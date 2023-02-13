function qSols = ik_student(x,y,q0,a1,a2,flag)
%ik_student Calculates the inverse kinematics for a 2-link, planar robot
%   Using inverse kinematics derived from the robot's geometry, the joint
%   coordinates needed to reach an end effector position of (x,y) for a
%   robot within link lengths a1 and a2, are found. Due to the presence of
%   multiple solutions (elbow-up, elbow-down), the best solution is
%   determined to be that closest to the previous position of the end
%   effector, provided as q0.
%   The flag is a binary input that determines which ik solution to return.
%   A true flag returns the 'best' solution, while a flase flag
%   deliberately selects the less desirable solution. For the purposes of
%   the problem set, set flag to be true.
xy2 = x^2 + y^2;
ik_candidate = [];

% STUDENTS: Write the inverse kinematic equations for the joint angles q1 
% and q2 here.
% q2 = ???
% q1 = ???

if ~isnan([q1,q2])
    ik_candidate = [ik_candidate;q1,q2];
end

% STUDENTS: Write the second solution for the inverse kinematic equations
% here.
% q2 = ???
% q1 = ???

if ~isnan([q1,q2])
    ik_candidate = [ik_candidate;q1,q2];
end

min_v = NaN;
best_q = NaN;
if flag == true
    for i = 1:size(ik_candidate,1)
        q = ik_candidate(i,:);
        v = sum(((q-q0)).^2);
        if isnan(min_v) || min_v > v
            min_v = v;
            best_q = q;
        end
    end
else
    for i = 1:size(ik_candidate,1)
        q = ik_candidate(i,:);
        v = sum(((q-q0)).^2);
        if isnan(min_v) || min_v < v
            min_v = v;
            best_q = q;
        end
    end
end
q1 = best_q(1);
q2 = best_q(2);
qSols = [q1;q2];
end


