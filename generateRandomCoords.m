function [coords] = generateRandomCoords(num_coords, origins, radius, test_site_size)
% GENERATE_RANDOM_COORDS Generate random coordinates avoiding circular areas
%
%   [coords] = generate_random_coords(num_coords, origins, radii)
%
%   Input Arguments:
%       num_coords: Number of random coordinates to generate
%       origins: [n x 2] array of origin coordinates [x, y]
%       radii: [n x 1] array of radii for circular areas
%
%   Output Arguments:
%       coords: [num_coords x 2] array of random coordinates [x, y]

% Initialize empty array for coordinates
    % max_attempts = 10000;
    % for attempt = 1:max_attempts
        coords = zeros(num_coords,2);
            for i = 1:num_coords
                while true
                    x = rand() * test_site_size;
                    y = rand() * test_site_size;
                    
                        if sqrt((x - origins(i,1))^2 + (y - origins(i,2))^2) > radius
                                coords(i, :) = [x, y];  
                                break
                        end
                    end
            end
            
        end
    %     error('Unable to find a suitable configuration for ego robot after %d attempts', max_attempts);
    % end