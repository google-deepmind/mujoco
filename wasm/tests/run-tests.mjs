import Jasmine from "jasmine"

const jasmine = new Jasmine()

jasmine.loadConfig({
  spec_dir: "tests",
  spec_files: ["**/*_test.ts"],
  random: false,
  stopSpecOnExpectationFailure: false,
})

jasmine.execute()
