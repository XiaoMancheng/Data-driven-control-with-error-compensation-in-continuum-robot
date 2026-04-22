clear; clc; close all;

% DDCEC means Data driven control with error compensation

%% Simulation settings
N  = 800;     % horizon
m  = 2;       % number of inputs/outputs
nu = 3;       % plant-side PFDL length
Le = 2;       % controller-side error regression length

% Shared plant-side MFAC parameters (kept the same as original baselines)
eta   = 0.5;
miu   = 1.0;
rho   = 0.5;
lamda = 0.01;

% Projection/reset parameters in original example
b1 = 0.2;
b2 = 0.5;
M  = 10 * b2;

% DDCEC parameters
mu_c          = 5.0;     % controller-side regularization
gamma_c       = 0.02;    % nominal compensation weight
beta_c        = 1.0;     % normalization factor of adaptive blending
err_filt      = 0.5;     % error filtering factor in [0,1)
psi_diag_init = -0.01;   % negative feedback initialization
psi_bound     = 0.8;     % projection bound of pseudo-gradient entries
du_limit      = 1.0;     % increment saturation for DDCEC
u_limit       = 20.0;    % absolute input saturation

%% Reference signal
k = 0:N+1;
yd = zeros(N+2, m);
for t = 1:N+2
    yd(t,1) = 5 * sin((t-1)/50) + 2 * cos((t-1)/20);
    yd(t,2) = 2 * sin((t-1)/50) + 5 * cos((t-1)/20);
end

%% Simulations
[y_cfdl, u_cfdl] = simulate_cfdl_mimo(N, yd, eta, miu, rho, lamda, b1, b2, M);
[y_pfdl, u_pfdl] = simulate_pfdl_mimo(N, yd, nu, eta, miu, rho, lamda, b1, b2, M);
[y_cdl,  u_cdl ] = simulate_cdl_pfdl_mimo_optimized(N, yd, nu, Le, eta, miu, rho, lamda, ...
    b1, b2, M, mu_c, gamma_c, beta_c, err_filt, psi_diag_init, psi_bound, du_limit, u_limit);

%% Metrics
metric_names = {'CFDL-MFAC'; 'PFDL-MFAC'; 'DDCEC'};
Y_all = {y_cfdl, y_pfdl, y_cdl};
U_all = {u_cfdl, u_pfdl, u_cdl};

RMSE = zeros(3,2);
IAE  = zeros(3,2);
RMSU = zeros(3,2);
for i = 1:3
    err = yd(1:N,:) - Y_all{i}(1:N,:);
    RMSE(i,:) = sqrt(mean(err.^2, 1));
    IAE(i,:)  = sum(abs(err), 1);
    RMSU(i,:) = sqrt(mean(U_all{i}(1:N,:).^2, 1));
end

T_metrics = table(metric_names, RMSE(:,1), RMSE(:,2), IAE(:,1), IAE(:,2), RMSU(:,1), RMSU(:,2), ...
    'VariableNames', {'Controller','RMSE_y1','RMSE_y2','IAE_y1','IAE_y2','RMS_u1','RMS_u2'});
disp(T_metrics);

%% Colors
c_ref  = [0 0 0];
c_cfdl = [0 0.4470 0.7410];
c_pfdl = [0.8500 0.3250 0.0980];
c_cdl  = [0.4660 0.6740 0.1880];

%% Plot y1
figure('Color','w');
plot(k(1:N), yd(1:N,1), 'Color', c_ref,  'LineWidth', 2.2); hold on;
plot(k(1:N), y_cfdl(1:N,1), 'Color', c_cfdl, 'LineWidth', 1.8);
plot(k(1:N), y_pfdl(1:N,1), 'Color', c_pfdl, 'LineWidth', 1.8);
plot(k(1:N), y_cdl(1:N,1),  'Color', c_cdl,  'LineWidth', 1.8);
grid on; box on;
xlabel('k'); ylabel('y_1');
legend({'Reference','CFDL-MFAC','PFDL-MFAC','DDCEC'}, 'Location', 'best');
title('Tracking performance of y_1');
xlim([0 N]);

%% Plot y2
figure('Color','w');
plot(k(1:N), yd(1:N,2), 'Color', c_ref,  'LineWidth', 2.2); hold on;
plot(k(1:N), y_cfdl(1:N,2), 'Color', c_cfdl, 'LineWidth', 1.8);
plot(k(1:N), y_pfdl(1:N,2), 'Color', c_pfdl, 'LineWidth', 1.8);
plot(k(1:N), y_cdl(1:N,2),  'Color', c_cdl,  'LineWidth', 1.8);
grid on; box on;
xlabel('k'); ylabel('y_2');
legend({'Reference','CFDL-MFAC','PFDL-MFAC','DDCEC'}, 'Location', 'best');
title('Tracking performance of y_2');
xlim([0 N]);

%% Plot u1
figure('Color','w');
plot(k(1:N), u_cfdl(1:N,1), 'Color', c_cfdl, 'LineWidth', 1.8); hold on;
plot(k(1:N), u_pfdl(1:N,1), 'Color', c_pfdl, 'LineWidth', 1.8);
plot(k(1:N), u_cdl(1:N,1),  'Color', c_cdl,  'LineWidth', 1.8);
grid on; box on;
xlabel('k'); ylabel('u_1');
legend({'CFDL-MFAC','PFDL-MFAC','DDCEC'}, 'Location', 'best');
title('Control input u_1');
xlim([0 N]);

%% Plot u2
figure('Color','w');
plot(k(1:N), u_cfdl(1:N,2), 'Color', c_cfdl, 'LineWidth', 1.8); hold on;
plot(k(1:N), u_pfdl(1:N,2), 'Color', c_pfdl, 'LineWidth', 1.8);
plot(k(1:N), u_cdl(1:N,2),  'Color', c_cdl,  'LineWidth', 1.8);
grid on; box on;
xlabel('k'); ylabel('u_2');
legend({'CFDL-MFAC','PFDL-MFAC','DDCEC'}, 'Location', 'best');
title('Control input u_2');
xlim([0 N]);

%% ===== Local functions =====
function [y, u] = simulate_cfdl_mimo(N, yd, eta, miu, rho, lamda, b1, b2, M)
    y = zeros(N+2, 2);
    u = zeros(N+2, 2);
    y(2,:) = [1 1];

    Phi = zeros(N+2, 2, 2);
    Phi(1,:,:) = [0.5 0; 0 0.5];
    Phi(2,:,:) = [0.5 0; 0 0.5];

    du = zeros(2,1);
    for k = 3:N
        Phi_prev = squeeze(Phi(k-1,:,:));

        row1 = Phi_prev(1,:).' + eta * (y(k,1) - y(k-1,1) - Phi_prev(1,:) * du) * du / (miu + du.' * du);
        if (abs(row1(1)) < b2) || (abs(row1(1)) > M) || (row1(1) < 0)
            row1(1) = 0.5;
        end
        if abs(row1(2)) > b1
            row1(2) = 0;
        end

        row2 = Phi_prev(2,:).' + eta * (y(k,2) - y(k-1,2) - Phi_prev(2,:) * du) * du / (miu + du.' * du);
        if abs(row2(1)) > b1
            row2(1) = 0;
        end
        if (abs(row2(2)) < b2) || (abs(row2(2)) > M) || (row2(2) < 0)
            row2(2) = 0.5;
        end

        Phi_k = [row1.'; row2.'];
        Phi(k,:,:) = Phi_k;

        e = (yd(k+1,:) - y(k,:)).';
        du_k = rho * (Phi_k.' * e) / (lamda + norm(Phi_k,2)^2);
        u(k,:) = u(k-1,:) + du_k.';

        y(k+1,:) = plant_mimo(y, u, k);
        du = (u(k,:) - u(k-1,:)).';
    end
end

function [y, u] = simulate_pfdl_mimo(N, yd, nu, eta, miu, rho, lamda, b1, b2, M)
    y = zeros(N+2, 2);
    u = zeros(N+2, 2);
    y(2,:) = [1 1];

    fai11 = zeros(N+2, nu); fai12 = zeros(N+2, nu);
    fai21 = zeros(N+2, nu); fai22 = zeros(N+2, nu);
    fai11(1:2,1) = 0.5;
    fai22(1:2,1) = 0.5;

    du1 = zeros(1, nu); du2 = zeros(1, nu);
    du  = [du1 du2].';

    for k = 3:N
        fai1_prev = [fai11(k-1,:) fai12(k-1,:)].';
        fai1 = fai1_prev + eta * (y(k,1) - y(k-1,1) - fai1_prev.' * du) * du / (miu + du.' * du);
        if (abs(fai1(1)) < b2) || (abs(fai1(1)) > M) || (fai1(1) < 0)
            fai1(1) = 0.5;
        end
        if abs(fai1(1+nu)) > b1
            fai1(1+nu) = 0;
        end
        fai11(k,:) = fai1(1:nu).';
        fai12(k,:) = fai1(nu+1:2*nu).';

        fai2_prev = [fai21(k-1,:) fai22(k-1,:)].';
        fai2 = fai2_prev + eta * (y(k,2) - y(k-1,2) - fai2_prev.' * du) * du / (miu + du.' * du);
        if abs(fai2(1)) > b1
            fai2(1) = 0;
        end
        if (abs(fai2(1+nu)) < b2) || (abs(fai2(1+nu)) > M) || (fai2(1+nu) < 0)
            fai2(1+nu) = 0.5;
        end
        fai21(k,:) = fai2(1:nu).';
        fai22(k,:) = fai2(nu+1:2*nu).';

        Phi1 = [fai11(k,1) fai12(k,1); fai21(k,1) fai22(k,1)];
        d = [yd(k+1,1) - y(k,1) - fai11(k,2:nu) * du1(1:nu-1).' - fai12(k,2:nu) * du2(1:nu-1).';
             yd(k+1,2) - y(k,2) - fai21(k,2:nu) * du1(1:nu-1).' - fai22(k,2:nu) * du2(1:nu-1).'];

        du_k = rho * (Phi1.' * d) / (lamda + norm(Phi1,2)^2);
        u(k,:) = u(k-1,:) + du_k.';
        y(k+1,:) = plant_mimo(y, u, k);

        for i = 1:nu
            if (k-i) <= 0
                du1(i) = 0; du2(i) = 0;
            else
                du1(i) = u(k-i+1,1) - u(k-i,1);
                du2(i) = u(k-i+1,2) - u(k-i,2);
            end
        end
        du = [du1 du2].';
    end
end

function [y, u] = simulate_cdl_pfdl_mimo_optimized(N, yd, nu, Le, eta, miu, rho, lamda, ...
    b1, b2, M, mu_c, gamma_c, beta_c, err_filt, psi_diag_init, psi_bound, du_limit, u_limit)

    y = zeros(N+2, 2);
    u = zeros(N+2, 2);
    y(2,:) = [1 1];

    fai11 = zeros(N+2, nu); fai12 = zeros(N+2, nu);
    fai21 = zeros(N+2, nu); fai22 = zeros(N+2, nu);
    fai11(1:2,1) = 0.5;
    fai22(1:2,1) = 0.5;

    e  = zeros(N+2, 2);
    ef = zeros(N+2, 2);

    Psi = zeros(N+2, 2, 2*Le);
    Psi_init = zeros(2, 2*Le);
    Psi_init(1,1) = psi_diag_init;
    Psi_init(2,2) = psi_diag_init;
    Psi(1,:,:) = Psi_init;
    Psi(2,:,:) = Psi_init;

    du1 = zeros(1, nu); du2 = zeros(1, nu);
    du  = [du1 du2].';

    for k = 3:N
        % ===== plant-side PFDL estimation (kept identical in structure) =====
        fai1_prev = [fai11(k-1,:) fai12(k-1,:)].';
        fai1 = fai1_prev + eta * (y(k,1) - y(k-1,1) - fai1_prev.' * du) * du / (miu + du.' * du);
        if (abs(fai1(1)) < b2) || (abs(fai1(1)) > M) || (fai1(1) < 0)
            fai1(1) = 0.5;
        end
        if abs(fai1(1+nu)) > b1
            fai1(1+nu) = 0;
        end
        fai11(k,:) = fai1(1:nu).';
        fai12(k,:) = fai1(nu+1:2*nu).';

        fai2_prev = [fai21(k-1,:) fai22(k-1,:)].';
        fai2 = fai2_prev + eta * (y(k,2) - y(k-1,2) - fai2_prev.' * du) * du / (miu + du.' * du);
        if abs(fai2(1)) > b1
            fai2(1) = 0;
        end
        if (abs(fai2(1+nu)) < b2) || (abs(fai2(1+nu)) > M) || (fai2(1+nu) < 0)
            fai2(1+nu) = 0.5;
        end
        fai21(k,:) = fai2(1:nu).';
        fai22(k,:) = fai2(nu+1:2*nu).';

        Phi1 = [fai11(k,1) fai12(k,1); fai21(k,1) fai22(k,1)];
        d = [yd(k+1,1) - y(k,1) - fai11(k,2:nu) * du1(1:nu-1).' - fai12(k,2:nu) * du2(1:nu-1).';
             yd(k+1,2) - y(k,2) - fai21(k,2:nu) * du1(1:nu-1).' - fai22(k,2:nu) * du2(1:nu-1).'];

        % ===== original plant-side PFDL control term =====
        du_pfdl = rho * (Phi1.' * d) / (lamda + norm(Phi1,2)^2);

        % ===== controller-side CDL term =====
        e(k,:)  = yd(k,:) - y(k,:);
        ef(k,:) = (1 - err_filt) * e(k,:) + err_filt * ef(k-1,:);
        zeta = [-ef(k,1); -ef(k,2); ef(k,1)-ef(k-1,1); ef(k,2)-ef(k-1,2)];

        Psi_prev = squeeze(Psi(k-1,:,:));
        Kpsi = kron(zeta * zeta.', Phi1.' * Phi1) + mu_c * eye(2 * 2 * Le);
        rhs  = Phi1.' * (d * zeta.') + mu_c * Psi_prev;
        vecPsi = Kpsi \ rhs(:);
        Psi_k  = reshape(vecPsi, 2, 2*Le);

        if any(~isfinite(Psi_k(:))) || norm(Psi_k, 'fro') > 20
            Psi_k = Psi_prev;
        end

        % projection and sign reset on leading diagonal entries
        Psi_k = max(min(Psi_k, psi_bound), -psi_bound);
        if Psi_k(1,1) > -1e-4
            Psi_k(1,1) = psi_diag_init;
        end
        if Psi_k(2,2) > -1e-4
            Psi_k(2,2) = psi_diag_init;
        end
        Psi(k,:,:) = Psi_k;

        du_c = Psi_k * zeta;
        gamma_eff = gamma_c / (1 + beta_c * norm(zeta, 2));

        % ===== fused control update =====
        du_k = du_pfdl + gamma_eff * du_c;
        du_k = max(min(du_k, du_limit), -du_limit);

        u(k,:) = u(k-1,:) + du_k.';
        u(k,:) = max(min(u(k,:), u_limit), -u_limit);

        y(k+1,:) = plant_mimo(y, u, k);

        for i = 1:nu
            if (k-i) <= 0
                du1(i) = 0; du2(i) = 0;
            else
                du1(i) = u(k-i+1,1) - u(k-i,1);
                du2(i) = u(k-i+1,2) - u(k-i,2);
            end
        end
        du = [du1 du2].';
    end
end

function y_next = plant_mimo(y, u, k)
    y1 = y(k,1);   y1m1 = y(k-1,1);   y1m2 = y(k-2,1);
    y2 = y(k,2);   y2m1 = y(k-1,2);   y2m2 = y(k-2,2);
    u1 = u(k,1);   u1m2 = u(k-2,1);
    u2 = u(k,2);   u2m1 = u(k-1,2);   u2m2 = u(k-2,2);

    y1_next = 2.5 * y1 * y1m1 / (1 + y1^2 + y2m1^2 + y1m2^2) ...
            + 0.7 * sin(0.5 * (y1 + y1m1)) * cos(0.5 * (y1 + y1m1)) ...
            + 0.09 * u1 * u2m1 + 1.2 * u1 + 1.6 * u1m2 + 0.5 * u2;

    y2_next = 5 * y2 * y2m1 / (1 + y2^2 + y1m1^2 + y2m2^2) ...
            + u2 + 1.1 * u2m1 + 1.4 * u2m2 + 0.5 * u1;

    y_next = [y1_next, y2_next];
end
