% draws the pose of a given homogeneous transformation matrix via quiver
function quiver_T(transform)
    quiver3(ones(3,1)*transform(1,4),ones(3,1)*transform(2,4),ones(3,1)*transform(3,4),transform(1,1:3)',transform(2,1:3)',transform(3,1:3)');
end