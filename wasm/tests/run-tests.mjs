import Jasmine from 'jasmine';

const jasmine = new Jasmine();

jasmine.loadConfig({
  spec_dir: 'tests',
  spec_files: ['**/*_test.ts'],
  jsLoader: 'import',
  random: false,
  stopSpecOnExpectationFailure: false,
});

try {
  console.log('Starting Jasmine test run...');
  const result = await jasmine.execute();
  console.log(`Jasmine test run finished. Status: ${result.overallStatus}`);

  if (result.overallStatus === 'failed') {
    process.exit(1);
  }

} catch (error) {
  console.error('Test runner script failed:', error);
  process.exit(1);
}

