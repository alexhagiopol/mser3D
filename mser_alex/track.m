classdef track < handle
    properties
        RCdata; %[X1, Y1; X2, Y2 ...]  All of the positions we've associated with this track. Each row is one measurmeent.
        COVdata; %[cov11, cov21, cov 31; cov12, cov22, cov 32...] all of the covariance measurements made. Each row is one measurement.
        RCvel; %velocity we associate with the object in htis track
        color; %The color we associate with this tracked object. A 3 item vector [R,G,B]
        n; %number of measurements we've made
        %"Tuning knobs" for particle filtering ans state estimatetion:
        numParticles = 200; %number of particles per object
        sigma = 5;
        sigma_dynamics = 2;
        particles;
        frames_for_v_est = 5; %use 5 frames to estimate velocity
        average_locations; 
        %Range for initial particle locations;
        rLowerBound = 200;
        rUpperBound = 400;
        cLowerBound = 100;
        cUpperBound = 350;
    end     
    methods
        function T = track
            %We assume we will have a max of 1000 measurements per object
            space = 1000;
            T.RCdata = zeros(2,space);
            T.COVdata = zeros(3,space);
            T.RCvel = zeros(1,2);
            T.color = zeros(1,3);
            T.n = 1; %number of measurements we've made so far
            %assume image is 640 x 480
            rPositions = randsample(T.rLowerBound:T.rUpperBound,T.numParticles,true);
            cPositions = randsample(T.cLowerBound:T.cUpperBound,T.numParticles,true);
            T.particles = [rPositions',cPositions', (1/T.numParticles)*ones(T.numParticles,1)];
            T.average_locations = [];
        end
        function add_msmt(T, msmt)
            %[r;c;cov1;cov2;cov3]
            T.RCdata(:,T.n) = msmt(1:2);
            T.COVdata(:,T.n) = msmt(3:5);             
            if T.n == T.frames_for_v_est
                %set r velocity after frames_for_v_est frames 
                T.set_vel((T.RCdata(1,T.frames_for_v_est) - T.RCdata(1,1))/T.frames_for_v_est,(T.RCdata(2,T.frames_for_v_est) - T.RCdata(2,1))/T.frames_for_v_est);
            end            
            T.n = T.n + 1;
        end
        function set_color(T,r,g,b)
            T.color(1) = r;
            T.color(2) = g;
            T.color(3) = b;
        end
        function set_vel(T,r,c)
            T.RCvel(1) = r;
            T.RCvel(2) = c;
        end
        function assoc_msmt(T, new_msmt) %new_msmt is an nx8 element column vector [r;c;c1;c2;c3;r;g;b] with one column for each measurement
        %This is the naive data association function. It selects the best measurement match and adds 
        %the associated data to the structures above.   
            global C;
            cur_msmt = [T.RCdata(1,T.n-1);T.RCdata(2,T.n-1);T.COVdata(1,T.n-1);T.COVdata(2,T.n-1);T.COVdata(3,T.n-1);T.color(1);T.color(2);T.color(3);];
            %           R                  C                cov1               cov2               cov3               R          G          B     
            bestIndex = 1;
            bestScore = -Inf;
            for i = 1:size(new_msmt,2)
                r = cast(C(round(new_msmt(1,i)),round(new_msmt(2,i)),1),'double');
                g = cast(C(round(new_msmt(1,i)),round(new_msmt(2,i)),2),'double');
                b = cast(C(round(new_msmt(1,i)),round(new_msmt(2,i)),3),'double');
                differences = abs([new_msmt(:,i);r;g;b] - cur_msmt);
                score = -1*mean(differences./abs(cur_msmt));
                if score > bestScore
                    bestIndex = i;
                    bestScore = score;
                end               
            end
            best_msmt = new_msmt(:,bestIndex);
            T.add_msmt(best_msmt(1:5));
        end
        function pos = get_position_estimate(T)
            %just return most recent measurement for now
            pos = T.RCdata(1:2,T.n-1);            
        end
        function plot_stuff(T)
            %just plots most recent measurements for now            
            %plot(T.RCdata(2,1:T.n-1),T.RCdata(1,1:T.n-1),'Color',[T.color]/400,'Marker','.','LineStyle','none'); %plot tracked path
            plot(T.particles(:,2),T.particles(:,1),'Color',[T.color]/350,'Marker','.','LineStyle','none'); %plot particle positions
            if size(T.average_locations,2) > 1 
                plot(T.average_locations(:,2),T.average_locations(:,1),'Color',[T.color]/500,'Marker','.','LineStyle','none'); %plot particle positions
            end
                
        end
        function update_particles(T)            
            if T.n > T.frames_for_v_est %make sure we've already estimated velocity
                %Dynamics update. Use the velocity plus some Gaussian noise to
                %predict next position. p(x_t+1|x_t)
                newRC = T.particles(:,1:2) + random('norm',0,T.sigma_dynamics,[T.numParticles,2]) + [T.RCvel(1)*ones(T.numParticles,1),T.RCvel(2)*ones(T.numParticles,1)];
                T.particles(:,1:2) = round(newRC);
                %Measurement update and particle weight update. p(z_t|x_t)                
                newWeights = exp(-1*T.difference_func()/(2*T.sigma));
                T.particles(:,3) = newWeights;
                T.particles(:,3) = T.particles(:,3)/sum(T.particles(:,3)); %normalize particle weights.
                %Prepare for resampling stage
                T.particles = sortrows(T.particles,3); %Sort in ...
                T.particles = flipdim(T.particles,1); %...descending order. Earlier particles have higher weights and will be more likely to be selected.
                new_particles = zeros(T.numParticles,3); %Make new set of particles that will replace current set
                i = 1;
                while i <= T.numParticles %Weighted random sampling. i keeps track of the number of particles we've added.
                    for j = 1:T.numParticles  %j traverses the set of particles in descending order of weight gives each one a chance to get chosen
                        if i > T.numParticles
                            break;
                        elseif rand(1,1) < T.particles(j,3)
                            new_particles(i,:) = T.particles(j,:);
                            i = i+1;
                        end
                    end
                end
                T.particles = new_particles; %update particle set with new particles
                T.particles(:,3) = T.particles(:,3)/sum(T.particles(:,3)); %normalize particle weights.
                %Update storage of average particle locations; this is the
                %running tally of the location estimates for our targets
                weighted_mean_r = sum(T.particles(:,1).*T.particles(:,3));
                weighted_mean_c = sum(T.particles(:,2).*T.particles(:,3));
                T.average_locations = [T.average_locations; weighted_mean_r, weighted_mean_c];
            end
        end        
        function difference_col_vec = difference_func(T) %computes a difference quantity between the most recent associated measurement 
            global C;
            rowDiff = abs(T.particles(:,1) - T.RCdata(1,T.n-1)*ones(T.numParticles,1))./abs(T.RCdata(1,T.n-1)*ones(T.numParticles,1));
            colDiff = abs(T.particles(:,2) - T.RCdata(2,T.n-1)*ones(T.numParticles,1))./abs(T.RCdata(2,T.n-1)*ones(T.numParticles,1));
            rDiff = zeros(T.numParticles,1);
            gDiff = zeros(T.numParticles,1);
            bDiff = zeros(T.numParticles,1);
            for i = 1:T.numParticles
                rDiff(i) = abs(cast(C(T.particles(i,1),T.particles(i,2),1),'double') - T.color(1))/T.color(1);
                gDiff(i) = abs(cast(C(T.particles(i,1),T.particles(i,2),2),'double') - T.color(2))/T.color(2);
                bDiff(i) = abs(cast(C(T.particles(i,1),T.particles(i,2),3),'double') - T.color(2))/T.color(3);
            end
            rVelDiff = abs(rowDiff - T.RCvel(1))/abs(T.RCvel(1));
            cVelDiff = abs(colDiff - T.RCvel(2))/abs(T.RCvel(2));
            difference_col_vec = (rowDiff + colDiff + rDiff + gDiff + bDiff + rVelDiff + cVelDiff);
            %pause
        end
    end
end









