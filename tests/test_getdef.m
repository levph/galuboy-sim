classdef test_getdef < matlab.unittest.TestCase
%TEST_GETDEF  Verify struct-field-with-default helper (Part B).

    methods (Test)

        function existingFieldReturned(tc)
            s = struct('foo', 42);
            tc.verifyEqual(getdef(s, 'foo', 99), 42);
        end

        function missingFieldReturnsDefault(tc)
            s = struct('bar', 1);
            tc.verifyEqual(getdef(s, 'foo', 99), 99);
        end

        function emptyFieldReturnsDefault(tc)
            s = struct('foo', []);
            tc.verifyEqual(getdef(s, 'foo', 99), 99);
        end

        function nonStructReturnsDefault(tc)
            tc.verifyEqual(getdef(42, 'foo', 99), 99);
        end

        function stringFieldWorks(tc)
            s = struct('name', "hello");
            tc.verifyEqual(getdef(s, 'name', "world"), "hello");
        end

    end
end
