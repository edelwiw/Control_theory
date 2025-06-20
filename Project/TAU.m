classdef TAU
    methods (Static)
        function U = get_controllability_matrix(A, B) 
            dim = size(A, 1);
            for i = 1:dim
                if i == 1
                    U = B;
                else
                    U = [U, A^(i - 1) * B];
                end
            end
        end

        function W = get_observability_matrix(A, C)
            dim = size(A, 1);
            for i = 1:dim
                if i == 1
                    W = C;
                else
                    W = [W; C * A^(i - 1)];
                end
            end
        end

        % Find the controller using Sylvester equation by its eigenvalues
        function [K, sigma] = FindControllerSylvester(A, B, Gamma)
            [Am, An] = size(A);
            [Bm, Bn] = size(B);
            Y = [1, 1, 1, 1]; % TODO: Adjust size based on your needs
            cvx_begin sdp 
                variable P(Am, Am);
                A * P - P * Gamma == B * Y;
            cvx_end
            K = -Y * inv(P);
            sigma = eig(A + B * K);
        end


        function [K, sigma] = FindControllerLMI(A, B, alpha)
            [Am, An] = size(A);
            [Bm, Bn] = size(B);

            cvx_begin sdp
                variable P(Am, An)
                variable Y(Bn, Bm)
                P > 0.0001 * eye(Am);
                P * A' + A * P + 2 * alpha * P + Y' * B' + B * Y <= 0;
            cvx_end

            K = Y / P;
            sigma = eig(A + B * K);
        end

        function [K, mu, sigma] = FindControllerLMIMin(A, B, x0, alpha) 
            [Am, An] = size(A);
            [Bm, Bn] = size(B);

            cvx_begin sdp
                variable P(Am, An)
                variable Y(Bn, Bm)
                variable mumu;
                minimize mumu;
                P > 0.000001 * eye(Am);
                P * A' + A * P + 2 * alpha * P + Y' * B' + B * Y <= 0;
                [P x0;
                x0' 1] > 0;
                [P Y';
                Y mumu * eye(Bn) ] > 0;
            cvx_end

            K = Y / P;
            mu = sqrt(mumu);
            sigma = eig(A + B * K);
        end

        function [L, sigma] = FindObserverSylvester(A, C, Gamma)
            [Am, An] = size(A);
            [Cm, Cn] = size(C);
            Y = [1, 1, 1, 1; 1, 1, 1, 1]'; % TODO: Adjust size based on your needs
            cvx_begin sdp
                variable Q(Am, An)
                Gamma * Q - Q * A == Y * C;
            cvx_end
            L = Q \ Y;
            sigma = eig(A + L * C);
        end

        function [L, sigma] = FindObserverLMI(A, C, alpha) 
            [Am, An] = size(A);
            [Cm, Cn] = size(C);

            cvx_begin sdp
                variable Q(Am, An)
                variable Y(Cn, Cm)
                Q > 0.0001 * eye(Am);
                A' * Q + Q * A + 2 * alpha * Q + C' * Y' + Y * C <= 0;        
            cvx_end

            L = Q \ Y;
            sigma = eig(A + L * C);
        end

        function [Q, Y] = FindReducedOrderObserver(A, C, Gamma)
            [Am, An] = size(A);
            [Cm, Cn] = size(C);
            Y = [1, 0; 1, 0];
            % 2x2 * 2x4 - 2x4 * 4x4 = 2x2 * 2x4 
            cvx_begin sdp
                variable Q(2, 4); % TODO: Adjust size based on your needs
                Gamma * Q - Q * A == Y * C;
            cvx_end
        end

        function [K, J] = FindLQRController(A, B, Q, R, x0, v)
            [P, ~, ~] = icare(A, B, Q, R / v);
            K = -R \ B' * P;
            J = x0' * P * x0;
        end

        function L = FindObserverKalman(A, C, Q, R)
            [P, ~, ~] = icare(A', C', Q, R / 1);
            L = -P * C' / R;
        end

    end
end