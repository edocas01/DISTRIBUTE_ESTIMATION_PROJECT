% plot the ellipse given a covariance matrix and a probability
function point_ellipse = compute_ellipse(p_i, cov_j, prob)
	% find the ellipse around pj with covariance covj and probability prob
    [V, D] = eig(cov_j * prob);
    t = linspace(0, 2*pi, 100);
    point_ellipse = [p_i(1); p_i(2)] + (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
end