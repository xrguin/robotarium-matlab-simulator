function [newStart, newGoal, interceptPoint] = generateInterceptingPath(refStart, refGoal, ...
    test_site_size, safeRadius)

    while true

        dir2 = refGoal - refStart;
        t = rand() * 0.5;

        interceptPoint = refStart + t * dir2;


        if (interceptPoint <= test_site_size)
            distance = calcDist(interceptPoint, refStart);
            newStart = generateRandomPointOnCircle(interceptPoint, distance, ...
                test_site_size);

            if calcDist(newStart, refStart) > safeRadius
                dir3 = interceptPoint - newStart;
                newGoal = interceptPoint + dir3;
                break;
            end
        end


    end

end


function point = generateRandomPointOnCircle(center, radius, test_site_size)
    while true
        % Generate a random angle
        theta = 2 * pi * rand();
        
        % Calculate point on circle
        x = center(1) + radius * cos(theta);
        y = center(2) + radius * sin(theta);
        
        % Check if point is within test site
        if x >= 0 && x <= test_site_size && y >= 0 && y <= test_site_size
            point = [x, y];
            break;
        end
    end
end
