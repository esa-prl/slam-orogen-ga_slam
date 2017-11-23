#!/usr/bin/env ruby

require 'orocos'
require 'readline'

include Orocos

Orocos.initialize

Orocos.run 'ga_slam::Task' => 'ga_slam' do
    ga_slam = Orocos::TaskContext.get 'ga_slam'
    ga_slam.configure
    ga_slam.start

    Readline::readline("Press enter to exit...\n") do
    end
end

