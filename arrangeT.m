function time_interval = arrangeT(path)
    n = size(path,1);
    time_interval = zeros(1,n-1);
    for i=1:n-1
        time_interval(i) = distance(path(i,:)',path(i+1,:)');
    end
end

function dist = distance(point1, point2) %column vector
    d = point1 - point2;
    dist = sqrt(d' * d);
end