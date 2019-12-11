function M = matrixCompute(A,xc,zS)

% perform svd on the convarance matrix of the point set A
if(size(A,1)~=2)
   A = A'; 
end

% construct the convariance matrix
K = size(A,2);
B = bsxfun(@minus, A, xc(:));  % BΪ����Aָ����Ȥ��xc������
B = B/zS;
M = 1/K*(B*B');

% B : 2 * n
% M = [ mean(x*x) mean(x*y)
%        mean(x*y) mean(y*y) ]