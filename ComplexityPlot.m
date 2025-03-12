%% Complexity plot
close all; clf;

data1 = [   
    1  1  NaN  0.6190  1.0031  0.000131  0.000823;
    1  2  NaN  0.4843  1.0320  0.000114  0.001719;
    2  3  8.46 0.4444  0.5158  0.000126  0.002845;
    3  4  7.96 0.3902  0.4710  0.000122  0.003900;
    4  5  7.96 0.3926  0.4832  0.000116  0.005101;
    5  6  9.45 0.3886  0.4852  0.000115  0.005934;
    6  7  9.95 0.3770  0.4888  0.000116  0.007011;
    7  8 11.94 0.3667  0.4833  0.000116  0.007851;
    8  9 12.44 0.3897  0.4955  0.000122  0.009394;
    9 10 13.93 0.3797  0.5215  0.000124  0.010897;
];

data2 = [
    1  1  NaN  0.6671  1.4378  0.000121  0.000711;
    1  2  NaN  0.6686  1.6664  0.000144  0.001700;
    2  3  8.46 0.4756  0.4756  0.000113  0.003230;
    3  4  7.96 0.3660  0.4596  0.000107  0.003767;
    4  5  7.96 0.4059  0.4861  0.000113  0.004575;
    5  6 12.94 0.5472  0.4602  0.000124  0.006160;
    6  7 14.43 0.5428  0.4780  0.000107  0.006300;
    7  8 20.40 0.5070  0.4678  0.000115  0.007600;
    8  9 22.89 0.5151  0.4667  0.000114  0.008763;
    9 10 23.88 0.6279  0.4818  0.000144  0.012367;
];
data3 = [
    1  1   NaN     1.5727  0.9742  1.77e-04   8.37e-04;
    1  2   NaN     1.5543  0.8384  9.29e-05   0.00105;
    2  3   11.94  1.5571  0.5787  1.28e-04   0.0022;
    3  4    7.96  1.6771  0.4786  1.48e-04   0.0033;
    4  5    9.95  1.7182  0.5012  1.62e-04   0.00452;
    5  6   11.94  3.5587  0.4860  0.000183333  0.005466667;
    6  7   10.95  1.5604  0.5107  0.000157143  0.005485714;
    7  8   11.44  1.5858  0.5129  0.0001625    0.007525;
    8  9   13.43  1.5767  0.5020  0.000166667  0.008044444;
    9 10   14.93  1.5988  0.5275  0.00019     0.01004
];

% Define labels for plots with LaTeX formatting
y_labels = {'Comm (\%)', ...
            'RMS$\left(\mathbf{e}_\mathrm{p}^{i}\right)$ (m)', ...
            'RMS$\left(\tilde{\mathbf{q}}^{i,j}\right)$ (m)', ...
            '$\bar{t}_p$ ($\mu$s)', ...
            '$\bar{t}_l$ (ms)'};
titles = {'Comm', ...
          'RMS$\left(\mathbf{e}_\mathrm{p}^{i}\right)$', ...
          'RMS$\left(\tilde{\mathbf{q}}^{i,j}\right)$', ...
          '$\bar{t}_p$', ...
          '$\bar{t}_l$'};
saves = {'Comm', ...
         'RMSep', ...
         'RMSel', ...
         'tp', ...
         'tl'};
        
columns = [3, 4, 5, 6, 7]; % Corresponding column indices in the dataset
col_multiplier = [1, 1, 1, 1e6, 1e3]; % Multipliers for each column

% Create figure with subplots
figure;
for i = 2:5
    fig = figure(i);
    set(fig, 'position', [0 0 400 200]); % Adjust height for multiple subplots
    hold on;
    plot(data1(:,2), data1(:,columns(i)) * col_multiplier(i), 'bo-', 'LineWidth', 1.5, 'MarkerSize', 8); hold on;
    plot(data2(:,2), data2(:,columns(i)) * col_multiplier(i), 'rx-', 'LineWidth', 1.5, 'MarkerSize', 8);
    plot(data3(:,2), data3(:,columns(i)) * col_multiplier(i), 'g*-', 'LineWidth', 1.5, 'MarkerSize', 8);
    hold off;
    xlabel('$M$','FontSize', 12, 'Interpreter', 'latex'); % X-axis label
    ylabel(y_labels{i}, 'FontSize', 12, 'Interpreter', 'latex'); % Y-axis label
%     title(['Comparison of ', titles{i}], 'FontSize', 14, 'Interpreter', 'latex'); % Title
    legend({'Proposed', 'Method2', 'Method3'}, 'FontSize', 12, 'Interpreter', 'latex', 'Location', 'Best'); % Legend
    grid on;
%     meann = {saves{i}, mean(data1(:,columns(i))*col_multiplier(i)),...
%             mean(data2(:,columns(i))*col_multiplier(i)),...
%             mean(data3(:,columns(i))*col_multiplier(i))};
%     disp(meann)
    
    saveas(fig, ['Figures/computational_' saves{i} '.png']);
    savefig(fig, ['Figures/computational_' saves{i} '.fig']); % Save as .fig
end
