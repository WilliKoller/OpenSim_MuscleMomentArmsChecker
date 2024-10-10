function  [Cycle,TimeGain]=normalizetimebase(signal,trigind)
% NORMALIZETIMEBASE : calculates a 0-100% timebase, ensemble-averages cyclic signals 
% [Cycle,TimeGain]=NormalizeTimeBase(signal,trigind)
% INPUT : signal: any one-dimensional array
%         trigind : array of indices, default: [1 length(signal)]
%                   should increase monotoneusly 
% PROCESS: calculates new points based on a 0-100% time base
%          by spline interpolation for each time interval
%
% OUTPUT:  if length(trigind)=2: Cycle [101 1] 
%          if length(trigind)>2: Cycle [101 Ncycles+2] 
%             Ncyles=length(trigind)-1, 
%             Ncycles+1: mean signal per point, i.e. ensemble averaged
%             Ncycles+2: stand.dev ensemble averaged points
%          TimeGain: (average) amplification/reduction of time-axis (i.e. 100/(samples/cycle)) 
%
% WARNING user should be aware of information loss in case of excessive downsampling

% AUTHOR(S) AND VERSION-HISTORY
% $ Ver 1.2 April 2003 (Jaap Harlaar VUmc Amsterdam) 

% Copyright 2000-2006 This program is free software under the terms of the
% GNU General Public License (www.gnu.org) 

if nargin < 2, trigind=[1 length(signal)]; end
if nargin < 1, return, end

% FFE  a check for validity of indices 

Cycle=[1:101]';
CycleLength=-101;
N=length(trigind)-1;
if N>1,
   for i=1:N,
      x=[trigind(i):trigind(i+1)]-trigind(i);
      CycleLength(i)=length(x);
      x=x*100/(trigind(i+1)-trigind(i));
      x=x+1;
      Cycle(:,i)=interp1(x',signal(trigind(i):trigind(i+1))',[1:101]','PCHIP');
   end
   tmp=mean(Cycle(:,1:N)');
   Cycle(:,N+1)=tmp';
   tmp=std(Cycle(:,1:N)');
   Cycle(:,N+2)=tmp';
   TimeGain=101/mean(CycleLength);
elseif N==1,
   x=[trigind(1):trigind(2)]-trigind(1);
   CycleLength=length(x);
   x=x.*100/(trigind(2)-trigind(1))+1;
   Cycle(:,1)=interp1(x',signal(trigind(1):trigind(2))',[1:101]','PCHIP');
   TimeGain=101/CycleLength;
end

return
% ============================================ 
% END ### NormalizeTimeBase ###