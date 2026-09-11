function ref = build_reference_trajectory(cfg)
%BUILD_REFERENCE_TRAJECTORY Object and contact reference trajectories.

n1 = round(0.18 * cfg.N);
n2 = round(0.60 * cfg.N);
n3 = cfg.N - n1 - n2;

% Short horizontal section at the beginning
x1 = linspace(-0.85, -0.72, n1);
y1 = 1.50 * ones(1, n1);

% Cubic Bezier section
s = linspace(0, 1, n2);
P0 = [-0.72; 1.50];
P1 = [-0.50; 1.88];
P2 = [-0.18; 2.02];
P3 = [ 0.00; 2.00];

w0 = (1-s).^3;
w1 = 3*(1-s).^2 .* s;
w2 = 3*(1-s) .* s.^2;
w3 = s.^3;
curve = P0*w0 + P1*w1 + P2*w2 + P3*w3;

% Final horizontal section
x3 = linspace(0.00, 0.50, n3);
y3 = 2.00 * ones(1, n3);

ref.xObj = [x1, curve(1,:), x3];
ref.yObj = [y1, curve(2,:), y3];
ref.xObj = ref.xObj(1:cfg.N);
ref.yObj = ref.yObj(1:cfg.N);

ref.phiL = zeros(1, cfg.N);
ref.phiR = ref.phiL + pi;

contactOffset = cfg.cubeSize/2;
ref.PL = [ref.xObj - contactOffset; ref.yObj];
ref.PR = [ref.xObj + contactOffset; ref.yObj];
end
