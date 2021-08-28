% Found on the book Graphics Gem V
% Chap III.1 - The Best Least-Squares Line Fit
% Great book, btw
% EDIT: Later I found more useful the output format Ax + By = C, so I
% commented out the [rho, theta] format, still works though
function [A, B, C, t, r] = fit_line_hesse(data)
    % Following on page 94 of the book, where he translates the points to
    % the origin subtracting for the mean:
    centroid = [mean(data(:, 1)), mean(data(:, 2))];
    data_aux = [data(:, 1) - centroid(1), data(:, 2) - centroid(2)];
    
    % Equation 14: here data_aux first is used to compute a' ...
    data_aux_2 = data_aux.^2;
    sum_data_aux = sum(data_aux_2); % sum columns
    a = sum_data_aux(1) - sum_data_aux(2);
    
    % ... and then b':
    b = 0;
    for i=1:size(data_aux, 1)
        b = b + data_aux(i, 1) * data_aux(i, 2);
    end
    
    % Equation 16 and sanity check!
    if (a == 0.0 && b == 0.0)
        rho = [];
        theta = [];
    else
        alpha = a;
        beta = 2 * b;
        gamma = sqrt(alpha^2 + beta^2);
        
        % Finally equation 17:
        theta = atan2(alpha - gamma, beta);
        rho = centroid(1) * cos(theta) + centroid(2) * sin(theta);
        if nargout > 3
            t = theta;
            r = rho;
        end
    end

    % Equation 23 and sanity check!
    if (a == 0 && b == 0)
        A = []; B = []; C = [];
    else
        A = 2 * b;
        B = -(a + sqrt(a^2 + 4*(b^2)));
        C = A*centroid(1) + B*centroid(2);
    end
end