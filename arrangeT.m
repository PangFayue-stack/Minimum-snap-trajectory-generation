function time_interval = arrangeT(path, T)
    n = size(path,1);
    sum = zeros(1,n);
    for i=2:n
        sum(i) = distance(path(i-1,:)',path(i,:)') + sum(i-1);
    end
    time_interval = sum/sum(n)*T;
end

function dist = distance(point1, point2) %column vector
    d = point1 - point2;
    dist = sqrt(d' * d);
end