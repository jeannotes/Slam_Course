% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);
  % here it give the poses, for every edges, I guess in struct "edge.fromIdx"
  % it is construct like (x y theta) -> (x y theta), all in one column
  % edge.toIdx behaves like this either, so we have all edges, instead of
  % finding corresponding edges, no such need

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    % compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    Z = v2t(edge.measurement);
    e = t2v( inv(Z) * (inv(x1) * x2) );
    Fx = Fx + e' * edge.information * e;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    % compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    Ti = v2t(x); Ri = Ti(1:3, 1:3); ti = Ti(1:2, 3);
    e = Ri' * ( l - ti) - edge.measurement;
    Fx = Fx + e' * edge.information * e;

    % the inside meaning is like this
    %           o(landmark)
    %          *
    %         *
    %        o(robot this time)
    % o(map start)
    % the 'x' and 'l' are all poses from the original start point
    % ( l - ti) is the translation matrix from robot to landmark(** part in picture)
    % but now it still have big theta, and (x, y) are actually good, 
    % for example robot's theta is 4, landmark's theta is 20, after
    % subtraction, we have 20-4, this is from robot to landmark
    % however, the struct "edge.measurement" is seeing landmark from robot's frame
    % we may also think like this: ( l - ti) should be the variable "edge.measurement"
    % well, what if robot have initial spinning angle, so we need to spin back.
    % again, in edge.measurement, it is pure, no robot's first spinning angle
  end

end
