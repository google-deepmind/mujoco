import Jasmine from "jasmine"
import glob from "fast-glob"
import { pathToFileURL } from "url"

const jasmine = new Jasmine()

jasmine.loadConfig({
  spec_dir: "tests",
  spec_files: ["**/*_test.ts"],
  random: false,
  stopSpecOnExpectationFailure: false,
})

const files = await glob("tests/**/*_test.ts")

for (const file of files) {
  await import(pathToFileURL(file).href)
}

jasmine.execute()
