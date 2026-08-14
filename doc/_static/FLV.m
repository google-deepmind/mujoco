% Copyright 2021 DeepMind Technologies Limited
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.


% Force-Length-Velocity function of MuJoCo muscle model
% Defaults: FLV(0.5, 1.6, 1.5, 1.3, 1.2)


function FLV(lmin, lmax, vmax, fpmax, fvmax)

    % derived quantities
  a = 0.5*(lmin+1);
  b = 0.5*(1+lmax);
  c = fvmax-1;

    % length and velocity ranges to plot
    LL = linspace(lmin, lmax, 51);
    VV = linspace(-vmax, vmax, 51);

    % length-passive
    FP = zeros(size(LL));
    for i=1:length(LL)
        L = LL(i);

        if L<=1
            FP(i) = 0;
        elseif L<=b
            x = (L-1)/(b-1);
            FP(i) = 0.25*fpmax*x*x*x;
        else
            x = (L-b)/(b-1);
            FP(i) = 0.25*fpmax*(1+3*x);
        end
    end

    % length-active
    FL = zeros(size(LL));
    for i=1:length(LL)
        L = LL(i);

        FL(i) = bump(L, lmin, 1, lmax) + 0.15*bump(L, lmin, 0.5*(lmin+0.95), 0.95);
    end

    % velocity-active
    FV = zeros(size(VV));
    for i=1:length(VV)
        V = VV(i)/vmax;

        if V<=-1
            FV(i) = 0;
        elseif V<=0
            FV(i) = (V+1)*(V+1);
        elseif V<=c
            FV(i) = fvmax - (c-V)*(c-V)/c;
        else
            FV(i) = fvmax;
        end
    end

    % plot length
    figure(1);
    clf;
    subplot(2,2,1);
    plot(LL, FL, 'r', 'linewidth', 1);
    hold on;
    plot(LL, 0.5*FL, 'b', 'linewidth', 1);
    plot(LL, FP, 'k', 'linewidth', 1);
    axis tight;
    xlabel('length (L0)');
    ylabel('force (F0)');
    text(0.9, 0.85, 'act = 1.0');
    text(0.9, 0.4, 'act = 0.5');
    text(1.3, 1.2, 'passive');
    box off;
    grid on;
    set(gca, 'xtick', [lmin 1 lmax], 'xticklabel', {'lmin', '1', 'lmax'}, ...
    'ytick', [0 1 fpmax], 'yticklabel', {'0', '1', 'fpmax'});

    % plot velocity
    subplot(2,2,2);
    set( plot(VV, FV, 'linewidth', 1), 'color', [.1 .5 .1]);
    axis tight;
    xlabel('velocity (L0/s)');
    ylabel('force (F0)');
    box off;
    grid on;
    set(gca, 'xtick', [-vmax 0 vmax], 'xticklabel', {'-vmax', '0', 'vmax'}, ...
    'ytick', [0 1 fvmax], 'yticklabel', {'0', '1', 'fvmax'});

    % plot full activation
    subplot(2,2,3);
    surf(LL, VV, FV'*FL + ones(size(VV))'*FP);
    axis tight;
    xlabel('length');
    ylabel('velocity');
    zlabel('force');
    title('act = 1.0');
    box off;
    set(gca, 'xtick', [lmin, 1, lmax], 'ytick', [-vmax, 0, vmax], 'ztick', [0, 1]);

    % plot half activation
    subplot(2,2,4);
    surf(LL, VV, 0.5*FV'*FL + ones(size(VV))'*FP);
    axis tight;
    xlabel('length');
    ylabel('velocity');
    zlabel('force');
    title('act = 0.5');
    box off;
    set(gca, 'xtick', [lmin, 1, lmax], 'ytick', [-vmax, 0, vmax], 'ztick', [0, 1]);
end


% skewed bump function: quadratic spline
function y = bump(L, A, mid, B)
    left = 0.5*(A+mid);
    right = 0.5*(mid+B);

    if (L<=A) || (L>=B)
        y = 0;
    elseif L<left
        x = (L-A)/(left-A);
        y = 0.5*x*x;
    elseif L<mid
        x = (mid-L)/(mid-left);
        y = 1-0.5*x*x;
    elseif L<right
        x = (L-mid)/(right-mid);
        y = 1-0.5*x*x;
    else
        x = (B-L)/(B-right);
        y = 0.5*x*x;
    end
end
