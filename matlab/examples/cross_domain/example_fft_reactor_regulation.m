function result = example_fft_reactor_regulation(varargin)
%EXAMPLE_FFT_REACTOR_REGULATION Deprecated example alias.
warning('ddpack:DeprecatedAlgorithmName', ...
    'Use example_penalty_free_time_reactor_regulation.');
result = example_penalty_free_time_reactor_regulation(varargin{:});
end
