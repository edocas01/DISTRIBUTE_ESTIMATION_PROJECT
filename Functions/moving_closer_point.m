% move a pointj closer to another pointi with a given covariancej
function [point_ellipse, new_pj] = moving_closer_point(p_i, p_j, cov_j, prob)
	
	% find the ellipse around pj with covariance covj and probability prob
    [V, D] = eig(cov_j * prob);
    t = linspace(0, 2*pi, 30);
    point_ellipse = [p_j(1); p_j(2)] + (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

	% find the closest point on the ellipse to pi
	p_i = [p_i(1); p_i(2)];
	[~,idx] = min(sum((point_ellipse - p_i).^2,1));
	new_pj = point_ellipse(:,idx);
end