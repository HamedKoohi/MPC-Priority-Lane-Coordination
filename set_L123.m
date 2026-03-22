function [set_L, n_set_L]=set_L123(params)
L_option=[0;1];
% n_set_L=2^N_step;
n_set_L=0;
for n1=1:2
%     for n2=1:2
        %         for n3=1:2
        n_set_L=n_set_L+1;
        %             set_L(n_set_L, :)=[L_option(n1),L_option(n2),L_option(n3)];
%         set_L(n_set_L, :)=[L_option(n1), L_option(n2)];
                    set_L(n_set_L, :)=L_option(n1);
        %         end
%     end
end
% set_L=ones(2, 1);