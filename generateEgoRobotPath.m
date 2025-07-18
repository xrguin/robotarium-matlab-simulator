function [egoStart, egoGoal] = generateEgoRobotPath(allyStart, enemyStart, interceptPoint, test_site_size, safeRadius)
    max_attempts = 10000;
    for attempt = 1:max_attempts
        % Calculate distance from robot2Start to interceptPoint
        distance = calcDist(interceptPoint, allyStart);
        
        vect_to_intercept = interceptPoint - enemyStart;
        vect_to_intercept = vect_to_intercept / norm(vect_to_intercept);
        
        angle = -pi * rand();
        egoStart = enemyStart + 2 * distance * vect_to_intercept * ...
            [cos(angle), -sin(angle); sin(angle), cos(angle)];
        
        
        % Check if egoStart is far enough from both robot2Start and robot3Start
        if norm(egoStart - allyStart) > safeRadius && norm(egoStart - enemyStart) > safeRadius
            % Calculate egoGoal
            dir = interceptPoint - egoStart;
            egoGoal = interceptPoint + dir;
            
            % Check if egoGoal is within the test site
            if all(egoGoal >= 0 & egoGoal <= test_site_size)
                return
            end
        end
    end
    error('Unable to find a suitable configuration for ego robot after %d attempts', max_attempts);
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
